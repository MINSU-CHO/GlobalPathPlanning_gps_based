#! /home/minsu/anaconda3/bin/python3
import time
import numpy as np
import rospy

from rocon_std_msgs.msg import StringArray
from hellocm_msgs.msg import vehiclestate
from std_msgs.msg import String

import os
import math
import csv

class GlobalPathPlanning:
    def __init__(self):
        rospy.init_node('global_path_planning', anonymous=True)

        self.pub_globalpath = rospy.Publisher('/globalpath', StringArray, queue_size=1)
        self.pub_policy = rospy.Publisher('/policypath', StringArray, queue_size=1)
        self.pub_localization = rospy.Publisher('/currentpath',String,queue_size=1)

        rospy.Subscriber('/vehiclestate', vehiclestate, self.callback_state)

        self.state = vehiclestate()
        self.state_recieved = False
        self.currentLane = ''
        self.msg_globalpath = StringArray()
        self.globalpath = []
        self.bool_pathplanning = False
        self.bool_localization = False
        self.policy = StringArray()
        self.msg_policy = StringArray()



        self.theta = 37 * np.pi / 180
        # self.bias = 0.7

        self.mapdata_path = '/home/minsu/linedata/'

        with open(self.mapdata_path + 'lanedata.txt') as f:
            self.lanelist = f.readlines()
        with open(self.mapdata_path + 'crossdata.txt') as f:
            self.crosslist = f.readlines()
        # with open(self.mapdata_path + 'waypoints.txt') as f:
        #     self.waypoint = f.readlines()
        f = open('test1.csv','r')
        self.waypoint = csv.reader(f)
        


    def callback_state(self, msg_state):
        self.state = msg_state
        self.state_recieved = True

    def check(self,xmin, xmax, ymin, ymax,bias):
        _x = self.state.x
        _y = self.state.y

        x = _x * math.cos(self.theta) - _y * math.sin(self.theta)
        y = _x * math.sin(self.theta) + _y * math.cos(self.theta)
        
        if x<= (xmax + bias) and x>=(xmin - bias):
            if y<(ymax+bias) and y>(ymin-bias):
                return True
        return False

    def findMyLane(self):
        for lane in self.lanelist:
            str = lane.split()
            name = str[0]
            xmin = float(str[1])
            xmax = float(str[2])
            ymin = float(str[3])
            ymax = float(str[4])
            if self.check(xmin,xmax,ymin,ymax,0.7):
                self.currentLane = name + self.findHeading(name)
                self.bool_pathplanning = True
                return True
        # print("FINDING ERROR")
        return False

    def findCrossSection(self):
        for cross in self.crosslist:
            str = cross.split()
            name = str[0]
            xmin = float(str[1])
            xmax = float(str[2])
            ymin = float(str[3])
            ymax = float(str[4])
            if self.check(xmin,xmax,ymin,ymax,-0.1):
                self.currentLane = name
                self.bool_pathplanning = True
                return True

    def findHeading(self,name):
        label = ['N','S']
        # bias = 37*np.pi / 180
        yaw = self.state.psi + self.theta
        if name[0] == 'H':
            if yaw >= -np.pi/2 and yaw <= np.pi/2:
                return label[1]
            else:
                return label[0]
        else:
            if yaw > np.pi/4 and yaw<np.pi/4 * 3:
                return label[1]
            else:
                return label[0]

    def laneindex(self,name):
        for i, line in enumerate(self.lanelist):
            str = line.split()
            if name == str[0]:
                # print(i)
                return i
        return 0        

    def trajectory_planning(self):
        temp_list=[]
        # for point in self.waypoint:
        #     str = point.split()
        #     self.state.x = (float(str[2]) - 126.89*np.pi/180) * 6371 * 1000 * math.cos(float(str[1]))
        #     self.state.y = (float(str[1]) - 37.58*np.pi/180) * 6371 * 1000
        #     if self.findMyLane():
        #         temp_list.append(self.currentLane[:3])
        #     elif self.findCrossSection():
        #         temp_list.append(self.currentLane)
        for point in self.waypoint:
            if point[0] != '#':
                self.state.x = (float(point[2]) - 126.89*np.pi/180) * 6371 * 1000 * math.cos(float(point[1]))
                self.state.y = (float(point[1]) - 37.58*np.pi/180) * 6371 * 1000
                if self.findMyLane():
                    temp_list.append(self.currentLane[:3])
                elif self.findCrossSection():
                    temp_list.append(self.currentLane)

        ## Eliminate dulicated lane
        print(temp_list)

        for v in temp_list:
            if v not in self.globalpath:
                self.globalpath.append(v)

        # ## Make global path -> [lane, cross, lane] form
        tmp = []
        for i, lane in enumerate(self.globalpath):
            tmp.append(lane)
            if i < len(self.globalpath)-1:
                next_lane = self.globalpath[i+1]
                if lane[0] != 'C' and next_lane[0] != 'C':
                    idx = self.laneindex(lane)
                    idx2 = self.laneindex(next_lane)
                    if self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[6]:
                        cross = self.lanelist[idx].split()[5]
                        

                    elif self.lanelist[idx].split()[6] == self.lanelist[idx2].split()[5]:
                        cross = self.lanelist[idx].split()[6]
                        

                    else:
                        cross = 'UNCLASSIFIED'

                    tmp.append(cross)
        self.globalpath = tmp

        if self.globalpath[-1][0] == 'C':
            self.globalpath.remove(self.globalpath[-1])

        for i, lane in enumerate(self.globalpath):
            if i < len(self.globalpath)-3:
                if lane[0] != 'C':
                    next_lane = self.globalpath[i+2]
                    idx = self.laneindex(lane)
                    idx2 = self.laneindex(next_lane)

                    if lane[0] == next_lane[0]: # H -> H or V -> V
                        if lane == 'H33':
                            if next_lane == 'H26':
                                self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                                self.globalpath[i] = self.globalpath[i] + 'S'
                            elif next_lane == 'H27'
                                self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                                self.globalpath[i] = self.globalpath[i] + 'S'


                        elif next_lane == 'H33':
                            if lane == 'H26':
                                self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                                self.globalpath[i] = self.globalpath[i] + 'N'
                            elif lane == 'H27'
                                self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                                self.globalpath[i] = self.globalpath[i] + 'N'

                        else:
                            self.globalpath[i+1] = self.globalpath[i+1] +'S' #straight
                            if self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[6]:
                                
                                self.globalpath[i] = self.globalpath[i] + 'N'

                            else:
                                self.globalpath[i] = self.globalpath[i] + 'S'

                    elif lane[0] == 'H' and next_lane[0] == 'V':
                        if self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[6]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i] = self.globalpath[i] + 'N'
                        elif self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                            self.globalpath[i] = self.globalpath[i] + 'N'
                        elif self.lanelist[idx].split()[6] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i] = self.globalpath[i] + 'S'
                        else:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                            self.globalpath[i] = self.globalpath[i] + 'S'
                    else: # V -> H
                        if self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[6]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                            self.globalpath[i] = self.globalpath[i] + 'N'
                        elif self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i] = self.globalpath[i] + 'S'
                        elif self.lanelist[idx].split()[6] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                            self.globalpath[i] = self.globalpath[i] + 'S'
                        else:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i] = self.globalpath[i] + 'N'

                

            elif i is len(self.globalpath)-3:
                
                if lane[0] != 'C':
                    next_lane = self.globalpath[i+2]
                    idx = self.laneindex(lane)
                    idx2 = self.laneindex(next_lane)
                    if lane[0] == next_lane[0]: # H -> H or V -> V
                        if lane == 'H33':
                            if next_lane == 'H26':
                                self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                                self.globalpath[i] = self.globalpath[i] + 'S'
                            elif next_lane == 'H27'
                                self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                                self.globalpath[i] = self.globalpath[i] + 'S'

                        elif next_lane == 'H33':
                            if lane == 'H26':
                                self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                                self.globalpath[i] = self.globalpath[i] + 'N'
                            elif lane == 'H27'
                                self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                                self.globalpath[i] = self.globalpath[i] + 'N'
                        else:
                            self.globalpath[i+1] = self.globalpath[i+1] +'S' #straight
                            if self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[6]:
                                self.globalpath[i] = self.globalpath[i] + 'N'
                                self.globalpath[i+2] = self.globalpath[i+2] + 'N'
                            else:
                                self.globalpath[i] = self.globalpath[i] + 'S'
                                self.globalpath[i+2] = self.globalpath[i+2] + 'S'

                    elif lane[0] == 'H' and next_lane[0] == 'V':
                        if self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[6]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i] = self.globalpath[i] + 'N'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'N'
                        elif self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i] = self.globalpath[i] + 'N'
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'S'
                        elif self.lanelist[idx].split()[6] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i] = self.globalpath[i] + 'S'
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'N'
                        else:
                            self.globalpath[i] = self.globalpath[i] + 'S'
                            self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'N'
                    else: # V -> H
                        if self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[6]:
                            self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                            self.globalpath[i] = self.globalpath[i] + 'N'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'N'
                        elif self.lanelist[idx].split()[5] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i] = self.globalpath[i] + 'S'
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'N'
                        elif self.lanelist[idx].split()[6] == self.lanelist[idx2].split()[5]:
                            self.globalpath[i] = self.globalpath[i] + 'S'
                            self.globalpath[i+1] = self.globalpath[i+1] + 'R'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'S'
                        else:
                            self.globalpath[i] = self.globalpath[i] + 'S'
                            self.globalpath[i+1] = self.globalpath[i+1] + 'L'
                            self.globalpath[i+2] = self.globalpath[i+2] + 'N'

                    
    def policy_update(self):
        if self.currentLane[0] != 'C' and self.currentLane in self.globalpath:
            idx = self.globalpath.index(self.currentLane)
            if idx < len(self.globalpath):
                self.policy = self.globalpath[idx:min(idx+3,len(self.globalpath))]
            # else:
            #     self.policy = self.globalpath[idx:]
            #     for i in range(len(self.globalpath) - idx):
            #         self.policy.append(self.globalpath[-1])

        self.msg_policy = self.policy




        
    def mainloop(self):
        r = rospy.Rate(100)
        self.trajectory_planning()
        self.msg_globalpath = self.globalpath
        # self.pub_globalpath.publish(self.msg_globalpath)
        self.state.x = 0
        self.state.y = 0
        # self.bool_pathplanning = False
        while not rospy.is_shutdown():
            self.bool_localization = False
            if self.state_recieved:
                self.pub_globalpath.publish(self.msg_globalpath)
                if self.findMyLane():
                    self.pub_localization.publish(self.currentLane)
                    # self.bool_localization = True
                    # print(self.currentLane)
                elif self.findCrossSection():
                    self.pub_localization.publish(self.currentLane)
                    # print(self.currentLane)
                else:
                    self.pub_localization.publish(self.currentLane)
                    # print("I dont knwo where Am I")

                if self.bool_pathplanning:
                    self.policy_update()
                    self.msg_policy = self.policy
                    self.pub_policy.publish(self.msg_policy)
            r.sleep()


if __name__ == "__main__":
    try:
        G = GlobalPathPlanning()
        G.mainloop()
    except rospy.ROSInterruptException:
        pass
        