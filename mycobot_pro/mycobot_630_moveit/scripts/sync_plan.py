#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import threading

import rospy
from sensor_msgs.msg import JointState

from pymycobot.elephantrobot import ElephantRobot


mc = None
latest_angles = None
lock = threading.Lock()


def callback(data):
    global latest_angles

    data_list = []

    for value in data.position:
        angle = round(math.degrees(value), 2)
        data_list.append(angle)

    # Joint Compensation (Based on your robotic arm definition)
    if len(data_list) > 1:
        data_list[1] -= 90
    if len(data_list) > 3:
        data_list[3] -= 90

    with lock:
        latest_angles = data_list


def control_loop():
    global latest_angles, mc

    rate = rospy.Rate(20)  # Control Frequency: 20 Hz (Adjustable: 10–30 Hz)

    while not rospy.is_shutdown():
        angles = None

        with lock:
            if latest_angles is not None:
                angles = latest_angles.copy()

        if angles is not None:
            try:
                # Key: Reduce speed to avoid trajectory accumulation.
                mc.write_angles(angles, 800)

            except Exception as e:
                print("Control error:", e)

        rate.sleep()


def listener():
    global mc

    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.1.191")
    port = rospy.get_param("~port", 5001)

    print("Connecting to robot:", ip, port)

    # init robot
    mc = ElephantRobot(ip, int(port))

    if not mc.start_client():
        print("Failed to connect robot")
        sys.exit(1)

    mc.set_speed(90)

    # Subscriber joint_states
    rospy.Subscriber("joint_states", JointState, callback)

    # start control thread
    thread = threading.Thread(target=control_loop)
    thread.daemon = True
    thread.start()

    print("Control loop started...")
    rospy.spin()


if __name__ == "__main__":
    listener()