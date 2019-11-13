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
from dead_reckoning import Dead_reckoning
import matplotlib.pyplot as plt
from statistics_listener import Odom_listener
from statistics_listener import Cone_listener
import rospkg
import datetime


L = 1.0#wheelbase

if __name__ == "__main__":
    rospy.init_node('dead_reckoning', anonymous = True)
    print "updating self position..."
    car_pose = Dead_reckoning()
    odom = Odom_listener()
    cone_detected = Cone_listener()
    rate = rospy.Rate(5) # 10hz
    rate.sleep()
    while not rospy.is_shutdown():
        car_pose.update()
        rate.sleep()
    rospack = rospkg.RosPack()
    now = datetime.datetime.now()
    path = rospack.get_path('srt_mapping') + "/map/" + str(now.year) + "-" + str(now.month) + "-" + str(now.day) + "-" + str(now.hour) + "-" + str(now.minute) + "-" + str(now.second) + "-" + str(now.microsecond)
    file = open(path + ".txt","w+")
    odom.form_statistics(file)
    cone_detected.form_statistics(file)
    odom.draw_odom()
    cone_detected.draw_detected_cones()
    file.close()
    
    
    plt.savefig(path + ".png")
