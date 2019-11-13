#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
from fusion.msg import map_points
import numpy as np
from cone_listener import Cone_listener
from visualization_msgs.msg import *


class Cone_temp():
    def __init__(self):
        self.marker_pub = rospy.Publisher("/cone_temp", Marker, queue_size = 10)
        self.marker = Marker()
        
        self.marker.header.frame_id = "/map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = "package://srt_mapping/meshes/cone.dae"
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1 
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.lifetime = rospy.Duration(0.1)

    def pub(self,iidd,x,y,color):
        self.marker.id = iidd
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0
        if color == "r":
            self.marker.color.r = 1.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0
            self.marker.color.a = 0.5 
        elif color == "b":
            self.marker.color.r = 0.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.6
            self.marker.color.a = 0.5 
            #print "here"
        elif color == "y":
            self.marker.color.r = 1.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
            self.marker.color.a = 0.5 
        else:
            self.marker.color.r = 1.0
            self.marker.color.g = 1.0
            self.marker.color.b = 1.0
            self.marker.color.a = 0.5
        self.marker_pub.publish(self.marker)

class Cone_configured():
    def __init__(self):
        self.marker_pub = rospy.Publisher("/cone_configured", Marker, queue_size = 10)
        self.marker = Marker()
        
        self.marker.header.frame_id = "/map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = "package://srt_mapping/meshes/cone.dae"
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1 
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.lifetime = rospy.Duration()

    def pub(self,iidd,x,y,color):
        self.marker.id = iidd
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0
        if color == "r":
            self.marker.color.r = 1.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0
            self.marker.color.a = 0.5 
        elif color == "b":
            self.marker.color.r = 0.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.6
            self.marker.color.a = 0.5 
            print "here"
        elif color == "y":
            self.marker.color.r = 1.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
            self.marker.color.a = 0.5 
        else:
            self.marker.color.r = 1.0
            self.marker.color.g = 1.0
            self.marker.color.b = 1.0
            self.marker.color.a = 0.5
        self.marker_pub.publish(self.marker)
        

class Cone_fusion():
    def __init__(self):
        self.raw_cones = []
        self.configured_cones = []
        self.config = 3
        self.threshold = 1.5
        self.update_threshold = 2
        self.cone_temp = Cone_temp()
        self.cone_configured = Cone_configured()
        self.configured_count = 0

    def Euc_dist(self,pos1_x,pos1_y,pos2_x,pos2_y):
        return ((pos1_x - pos2_x)**2 + (pos1_y - pos2_y)**2)**0.5

    def push(self,global_cone,timestamp):
        for i in range(0,len(self.configured_cones)):
            if self.Euc_dist(self.configured_cones[i].mean_x,self.configured_cones[i].mean_y,global_cone.x,global_cone.y) < self.threshold:
                self.configured_cones[i].mean_x = (self.configured_cones[i].mean_x)*self.configured_cones[i].update_round + global_cone.x
                self.configured_cones[i].mean_y = (self.configured_cones[i].mean_y)*self.configured_cones[i].update_round + global_cone.y
                self.configured_cones[i].color = global_cone.color
                self.configured_cones[i].update_time = timestamp
                self.configured_cones[i].update_round += 1
                self.configured_cones[i].mean_x = self.configured_cones[i].mean_x /  self.configured_cones[i].update_round
                self.configured_cones[i].mean_y = self.configured_cones[i].mean_y /  self.configured_cones[i].update_round
                return

        for i in range(0,len(self.raw_cones)):
            if self.Euc_dist(self.raw_cones[i].mean_x,self.raw_cones[i].mean_y,global_cone.x,global_cone.y) < self.threshold:
                self.raw_cones[i].mean_x = (self.raw_cones[i].mean_x)*self.raw_cones[i].update_round + global_cone.x
                self.raw_cones[i].mean_y = (self.raw_cones[i].mean_y)*self.raw_cones[i].update_round + global_cone.y
                self.raw_cones[i].color = global_cone.color
                self.raw_cones[i].update_time = timestamp
                self.raw_cones[i].update_round += 1
                self.raw_cones[i].mean_x = self.raw_cones[i].mean_x / self.raw_cones[i].update_round
                self.raw_cones[i].mean_y = self.raw_cones[i].mean_y / self.raw_cones[i].update_round
                return
        new_raw = map_points()
        new_raw.mean_x = global_cone.x
        new_raw.mean_y = global_cone.y
        new_raw.update_time = timestamp
        new_raw.color = global_cone.color
        new_raw.update_round = 1
        self.raw_cones.append(new_raw)

    def update(self,timestamp):
        jj = len(self.raw_cones) - 1
        while jj >= 0:
            if self.raw_cones[jj].update_round >= self.config:
                self.configured_cones.append(self.raw_cones[jj])
                del self.raw_cones[jj]
            elif abs(self.raw_cones[jj].update_time - timestamp) > self.update_threshold:
                del self.raw_cones[jj]
            else:
                self.cone_temp.pub(jj,self.raw_cones[jj].mean_x,self.raw_cones[jj].mean_y,self.raw_cones[jj].color)
            jj -= 1 

        jj = len(self.configured_cones) - 1
        while jj >= 0:
            if abs(self.configured_cones[jj].update_time - timestamp) > self.update_threshold:
                self.cone_configured.pub(self.configured_count,self.configured_cones[jj].mean_x,self.configured_cones[jj].mean_y,self.configured_cones[jj].color)
                self.configured_count += 1
                del self.configured_cones[jj]
            else:
                self.cone_temp.pub(jj+len(self.raw_cones),self.configured_cones[jj].mean_x,self.configured_cones[jj].mean_y,self.configured_cones[jj].color) 
            jj -= 1
        #print "configured == " + str(len(self.configured_cones))
            
