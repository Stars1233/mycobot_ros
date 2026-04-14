#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
read_joint_angles.py - 读取Gazebo中mycobot450pro的关节角度
"""

import rospy
import math
from sensor_msgs.msg import JointState

class JointAngleReader:
    def __init__(self):
        rospy.init_node('joint_angle_reader', anonymous=True)
        
        # 关节名称列表
        self.joint_names = [
            "joint1",
            "joint2", 
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper_controller"
        ]
        
        # 存储最新的关节角度
        self.current_angles = {}
        
        # 订阅关节状态话题
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        
        rospy.loginfo("关节角度读取器已启动，正在监听 /joint_states 话题...")
        
    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        # 更新关节角度字典
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                # 将弧度转换为度
                angle_deg = math.degrees(position)
                self.current_angles[name] = angle_deg
        
        # 打印当前所有关节角度
        self.print_angles()
    
    def print_angles(self):
        """打印关节角度信息"""
        print("\n" + "="*50)
        print("MyCobot Pro 450 当前关节角度:")
        print("="*50)
        
        for joint_name in self.joint_names:
            if joint_name in self.current_angles:
                angle = self.current_angles[joint_name]
                print(f"{joint_name:20}: {angle:8.2f}°")
        
        print("="*50)
    
    def get_joint_angle(self, joint_name):
        """获取指定关节的角度"""
        return self.current_angles.get(joint_name, None)
    
    def get_all_angles(self):
        """获取所有关节角度"""
        return self.current_angles.copy()

def main():
    try:
        # 创建关节角度读取器
        reader = JointAngleReader()
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号，程序退出")

if __name__ == "__main__":
    main()