#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
import numpy as np

class Srt_Control_listener():
    def __init__(self):
        initial_control = rospy.wait_for_message('srt_control',Srt_Control)
        self.latest_time = rospy.get_time()
        self.control_list = []
        self.time_list = []
        self.control_list.append(initial_control)
        self.time_list.append(0.0)
        rospy.Subscriber("/srt_control", Srt_Control, self.callback)
        
    def callback(self,data):
        time = rospy.get_time()
        self.time_list.append(self.time_list[-1] + time - self.latest_time)
        self.latest_time = time
        self.control_list.append(data)

    def get_control(self,index):
        if len(self.control_list) > index:
            return self.control_list[index]
        else:
            return []

    def get_time(self,index):
        #requires the index to be valid
        return self.time_list[index]
        
