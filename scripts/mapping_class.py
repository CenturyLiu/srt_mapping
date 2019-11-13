#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from fusion.msg import cone_pos
from fusion.msg import cone_pos_whole
from fusion.msg import Srt_Control
import numpy as np
import math


class Mapping():
    def __init__(self):
        rospy.Subscriber("/cone_detected", cone_pos_whole, self.callback)
        self.cone_detected = []

    def callback(self,data):
        rospy.loginfo("callback for mapping")
        self.cone_detected.append(data)
        print data

    def haha(self):
        return self.cone_detected
