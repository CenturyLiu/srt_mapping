#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
from control_listener import Srt_Control_listener
from visualization_msgs.msg import *
import numpy as np
import math
import matplotlib.pyplot as plt
from cone_listener import Cone_listener
from cone_fusion import Cone_fusion
from cone_fusion import Cone_configured
from cone_fusion import Cone_temp
from cone_mapping import Cone_mapping

L = 1.0#wheelbase

class Pose():
    def __init__(self,iidd,x,y,yaw):
        #print "x = "
        #print x
        #print "y = "
        #print y
        self.marker_pub = rospy.Publisher("/pos_visual", Marker, queue_size = 10)
        marker = Marker()
        marker.id = iidd
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.ARROW
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = -0.25
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.5
 
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration()
        self.marker_pub.publish(marker)


class Dead_reckoning():
    def __init__(self):
        self.time_list = [0.0]
        self.x_list = [0.0]
        self.y_list = [0.0]
        self.yaw_list = [0.0]
        self.index = 1
        self.control_listener = Srt_Control_listener()
        self.latest_control = self.control_listener.get_control(0)
        self.latest_time = self.control_listener.get_time(0)
        self.cone_mapping = Cone_mapping()

    def update(self):
        new_control = self.control_listener.get_control(self.index)
        if new_control != []:
            new_time = self.control_listener.get_time(self.index)
            self.index += 1 
            denominator = np.tan(math.radians(self.latest_control.steer_angle))
            R = 1000000000000.0
            if denominator != 0:
                R = L / denominator 
            #R = L / denominator
            theta = self.latest_control.linear_velocity * (new_time - self.latest_time) / R
            #print "theta == " + str(theta)
            yaw = self.yaw_list[-1]
            x1 = self.x_list[-1] + R*np.sin(theta)*np.cos(yaw)-R*(1-np.cos(theta))*np.sin(yaw)
            y1 = self.y_list[-1] + R*np.sin(theta)*np.sin(yaw)+R*(1-np.cos(theta))*np.cos(yaw)
            yaw1 = yaw + math.atan2(1-np.cos(theta),np.sin(theta))
            #print "x1 == " + str(x1)
            #print "y1 == " + str(y1)
            #print "yaw1 == " + str(yaw1)
            self.cone_mapping.get_local_cones()
            self.cone_mapping.local2map(self.x_list[-1],self.y_list[-1],self.yaw_list[-1],self.latest_time,new_time,self.latest_control)
            self.latest_control = new_control
            self.x_list.append(x1)
            self.y_list.append(y1)
            self.yaw_list.append(yaw1)
            self.latest_time = new_time
            self.time_list.append(new_time)
            pose = Pose(self.index - 1, x1, y1, yaw1)
            #plt.plot(self.x_list[-1],self.y_list[-1],".")
            
            

    def plot(self):
        plt.plot(self.x_list[-1],self.y_list[-1])
        

if __name__ == "__main__":
    rospy.init_node('dead_reckoning', anonymous = True)
    print "updating self position..."
    car_pose = Dead_reckoning
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        car_pose.update
        rate.sleep()
    


