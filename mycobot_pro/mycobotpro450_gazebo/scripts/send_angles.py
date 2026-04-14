#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_angles.py - 终端发送角度到Gazebo机械臂
用法: 
  python3 send_angles.py 0 45 0 0 0 0       # 发送6个关节角度(度)
  python3 send_angles.py --rad 0 0.785 0 0 0 0  # 发送弧度
"""

import sys
import math
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_angles(angles_deg, duration=2.0):
    """发送角度到Gazebo机械臂"""
    rospy.init_node('send_angles', anonymous=True)
    
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(0.5)  # 等待发布器连接
    
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    
    # 转换为弧度
    angles_rad = [math.radians(a) for a in angles_deg]
    
    # 创建轨迹消息
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = joint_names
    
    pt = JointTrajectoryPoint()
    pt.positions = angles_rad
    pt.velocities = [0.0] * 6
    pt.time_from_start = rospy.Duration(duration)
    traj.points = [pt]
    
    # 发布
    pub.publish(traj)
    print(f"✅ 已发送角度: {angles_deg} (度)")
    print(f"   弧度值: {[round(a, 4) for a in angles_rad]}")
    
    rospy.sleep(0.5)

def main():
    if len(sys.argv) < 7:
        print("用法: python3 send_angles.py j1 j2 j3 j4 j5 j6")
        print("示例: python3 send_angles.py 0 45 0 0 0 0")
        print("      python3 send_angles.py --rad 0 0.785 0 0 0 0")
        sys.exit(1)
    
    # 检查是否使用弧度
    if sys.argv[1] == '--rad':
        angles = [float(a) for a in sys.argv[2:8]]
        angles_deg = [math.degrees(a) for a in angles]
    else:
        angles_deg = [float(a) for a in sys.argv[1:7]]
    
    send_angles(angles_deg)

if __name__ == "__main__":
    main()