#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
import numpy as np
from visualization_msgs.msg import *
import matplotlib.pyplot as plt

class Odom_listener():
    def __init__(self):
        self.odom_x = []
        self.odom_y = []
        rospy.Subscriber("/pos_visual", Marker, self.callback)

    def callback(self,data):
        self.odom_x.append(data.pose.position.x)
        self.odom_y.append(data.pose.position.y)
    
    def draw_odom(self):
        plt.plot(self.odom_x,self.odom_y,'g.')

    def form_statistics(self,target):
        x_str = "x = [" + str(self.odom_x[0])
        y_str = "y = [" + str(self.odom_y[0])
        for i in range(1,len(self.odom_x)):
            x_str += " " + str(self.odom_x[i])
            y_str += " " + str(self.odom_y[i])
        x_str += "]\n"
        y_str += "]\n"
        target.write("Odom checkpoints:\n")
        target.write(x_str)
        target.write(y_str)
        target.write("....................\n")
        
            

class Cone_listener():
    def __init__(self):
        self.cone_x = []
        self.cone_y = []
        self.r = []
        self.g = []
        self.b = []
        rospy.Subscriber("/cone_configured", Marker, self.callback)

    def callback(self,data):
        self.cone_x.append(data.pose.position.x)
        self.cone_y.append(data.pose.position.y)
        self.r.append(data.color.r)
        self.g.append(data.color.g)
        self.b.append(data.color.b)
    
    def draw_detected_cones(self):
        for i in range(0,len(self.cone_x)):
            plt.plot(self.cone_x[i],self.cone_y[i],marker = "o",color = (self.r[i],self.g[i],self.b[i]))

    def form_statistics(self,target):
        x_str = "x = [" + str(self.cone_x[0])
        y_str = "y = [" + str(self.cone_y[0])
        for i in range(1,len(self.cone_x)):
            x_str += " " + str(self.cone_x[i])
            y_str += " " + str(self.cone_y[i])
        x_str += "]\n"
        y_str += "]\n"
        target.write("Cones detected:\n")
        target.write(x_str)
        target.write(y_str)
        target.write("....................\n")
        


