#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
from fusion.msg import map_points
import numpy as np
from cone_listener import Cone_listener
from cone_fusion import Cone_fusion
from cone_fusion import Cone_configured
from cone_fusion import Cone_temp
from visualization_msgs.msg import *
import math

tf_lidar_base = 0.2
L = 1

class Cone_mapping():
    def __init__(self):
        self.cone_listen = Cone_listener()
        self.initial_time = self.cone_listen.get_initial_time()
        self.local_cones = []#array of cones that cannot be transformed into map frame
        self.cone_fusion = Cone_fusion() 

    def get_local_cones(self):
        temp = self.cone_listen.get_cones()
        for cone in temp:
            cone.stamp -= self.initial_time
        self.local_cones.extend(temp)
        
    def local2map(self,odom_x,odom_y,odom_yaw,time_begin,time_end,latest_control):
        #Effects: transform the cone pose from local to map frame
        #         only those cones within the given time range [time_begin,time_end) will be transformed.
        #print "time_begin == " + str(time_begin) + "time_end == " + str(time_end)
        denominator = np.tan(math.radians(latest_control.steer_angle))
        R = 1000000000000.0
        if denominator != 0:
            R = L / denominator 
        #R = L / denominator
        i = 0
        for i in range(0,len(self.local_cones)):
            if self.local_cones[i].stamp < time_begin:
                continue
            if self.local_cones[i].stamp >= time_end:
                i = i - 1
                break
            #print "timestamp == " + str(self.local_cones[i].stamp)
            theta = latest_control.linear_velocity * (self.local_cones[i].stamp - time_begin) / R
            x1 = odom_x + R*np.sin(theta)*np.cos(odom_yaw)-R*(1-np.cos(theta))*np.sin(odom_yaw)
            y1 = odom_y + R*np.sin(theta)*np.sin(odom_yaw)+R*(1-np.cos(theta))*np.cos(odom_yaw)
            yaw1 = odom_yaw + math.atan2(1-np.cos(theta),np.sin(theta))
            for j in range(0, len(self.local_cones[i].cones)):
                global_cone = cone_pos()
                global_cone.x = x1 + (self.local_cones[i].cones[j].x+tf_lidar_base)*np.cos(yaw1) - self.local_cones[i].cones[j].y*np.sin(yaw1)
                global_cone.y = y1 + (self.local_cones[i].cones[j].x+tf_lidar_base)*np.sin(yaw1) - self.local_cones[i].cones[j].y*np.cos(yaw1)    
                global_cone.color = self.local_cones[i].cones[j].color
                '''
                if global_cone.x < 0:
                    print "----------"
                    print "strange x == " + str(global_cone.x) + " y == " + str(global_cone.x)
                    print "R == " + str(R) + "theta == " + str(theta)
                    print "odom_x == " + str(odom_x) + "odom_y == " + str(odom_y) + "odom_yaw == " + str(odom_yaw)
                    print "timestamp == " + str(self.local_cones[i].stamp) + " time_begin == " + str(time_begin)
                '''
                self.cone_fusion.push(global_cone,self.local_cones[i].stamp)#the Cone_fusion container will generate the map and publish the data
        del self.local_cones[0:i+1]
        self.cone_fusion.update(time_end)
                
