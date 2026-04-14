#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from socket import *
import math
import sys
import time
from multiprocessing import Lock

import rospy
from sensor_msgs.msg import JointState

from pymycobot.elephantrobot import ElephantRobot

global mc


def callback(data):
    """callback function"""
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 3)
        data_list.append(radians_to_angles)
    
    data_list[1] = round(data_list[1] - 90, 3)
    data_list[3] = round(data_list[3] - 90, 3)
        
    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    mc.write_angles(data_list, 800)

def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.1.191")
    port = rospy.get_param("~port", 5001)
    print (ip, port)
    mc = ElephantRobot(ip, int(port))
    # START CLIENT
    res = mc.start_client()
    if not res:
        # print('res:', res)
        sys.exit(1)

    mc.set_speed(90)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    print ("sping ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
