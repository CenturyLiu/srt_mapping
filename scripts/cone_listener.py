#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
import numpy as np

class Cone_listener():
    def __init__(self):
        initial_cone = rospy.wait_for_message('/cone_detected',cone_pos_whole)
        self.cone_list = []
        self.cone_list.append(initial_cone)
        self.initial_time = initial_cone.stamp
        rospy.Subscriber("/cone_detected", cone_pos_whole, self.callback)
        
    def callback(self,data):
        self.cone_list.append(data)

    def get_initial_time(self):
        return self.initial_time

    def get_cones(self):
        ret = self.cone_list
        self.cone_list = []
        return ret
