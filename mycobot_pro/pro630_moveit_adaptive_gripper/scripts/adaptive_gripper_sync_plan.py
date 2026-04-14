#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from socket import *
import math
import sys
import threading

import rospy
from sensor_msgs.msg import JointState

from pymycobot.elephantrobot import ElephantRobot

mc = None

latest_arm = None
latest_gripper = None

lock = threading.Lock()


def callback(data):
    """"Callback Function: Updates Data Only (No Control Logic!)"""
    global latest_arm, latest_gripper
    
    arm_list = []
    gripper_list = []
    gripper_value = 0
    
    for index, value in enumerate(data.position):
        if index < 6:
            radians_to_angles = round(math.degrees(value), 3)
            arm_list.append(radians_to_angles)
        else:
            # Map Adaptive gripper value to 0-100 scale
            gripper_list.append(value)
            
    gripper_rad = gripper_list[0]
    # clamp
    gripper_rad = max(-1.11, min(0.0, gripper_rad))
    # linear mapping: [-1.11, 0] -> [0, 100]
    gripper_value = int((gripper_rad + 1.11) / 1.11 * 100)
    
    # Joint Compensation
    if len(arm_list) > 1:
        arm_list[1] = round(arm_list[1] - 90, 3)
    if len(arm_list) > 3:
        arm_list[3] = round(arm_list[3] - 90, 3)
        
    with lock:
        latest_arm = arm_list
        latest_gripper = gripper_value

def control_loop():
    """Control Thread: Fixed-Frequency Transmission (Core)"""
    global latest_arm, latest_gripper, mc

    rate = rospy.Rate(20)  # 20 HZ

    last_gripper = None

    while not rospy.is_shutdown():
        arm = None
        grip = None

        # Retrieve Latest Data
        with lock:
            if latest_arm is not None:
                arm = latest_arm.copy()
            grip = latest_gripper

        # robot control
        if arm is not None:
            try:
                # Reduce speed to prevent accumulation.
                mc.write_angles(arm, 800)
            except Exception as e:
                print("Arm control error:", e)

        # Gripper Control (Sent only when changed)
        if grip is not None and grip != last_gripper:
            try:
                mc.set_gripper_value(grip, 100)
                last_gripper = grip
            except Exception as e:
                print("Gripper control error:", e)

        rate.sleep()

def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.1.159")
    port = rospy.get_param("~port", 5001)
    
    print("Connecting to robot:", ip, port)
    
    # init robot
    mc = ElephantRobot(ip, int(port))

    if not mc.start_client():
        print("Failed to connect robot")
        sys.exit(1)

    mc.set_speed(90)
    mc.set_gripper_mode(0)

    # subscriber  joint_states
    rospy.Subscriber("joint_states", JointState, callback)

    # start control thread
    thread = threading.Thread(target=control_loop)
    thread.daemon = True
    thread.start()

    print("Control loop started...")
    rospy.spin()


if __name__ == "__main__":
    listener()
