#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
j4_offset_test.py - J4关节偏移测试
验证10度偏移补偿是否有效
"""

import rospy
import math
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot import Pro450Client

class J4OffsetTest:
    def __init__(self):
        rospy.init_node('j4_offset_test', anonymous=True)
        
        # 关节名称
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Gazebo角度
        self.gazebo_angles = {}
        
        # Pro450连接
        self.mc = None
        
        # 订阅Gazebo状态
        rospy.Subscriber("/joint_states", JointState, self.gazebo_callback)
        
        # 发布器
        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        
        rospy.loginfo("🧪 J4偏移测试器已启动")
    
    def gazebo_callback(self, msg):
        """处理Gazebo关节状态"""
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.gazebo_angles[name] = math.degrees(position)
    
    def connect_pro450(self):
        """连接Pro450"""
        try:
            self.mc = Pro450Client("192.168.0.232", 4500)
            time.sleep(1.0)
            self.mc.power_on()
            time.sleep(1.0)
            return True
        except Exception as e:
            rospy.logerr(f"连接失败: {e}")
            return False
    
    def compensate_j4_angle(self, gazebo_angles, offset=-10.0):
        """补偿J4角度"""
        compensated = gazebo_angles.copy()
        compensated[3] += offset  # joint4索引为3
        return compensated
    
    def send_to_gazebo(self, angles):
        """发送角度到Gazebo"""
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(a) for a in angles]
        pt.time_from_start = rospy.Duration(2.0)
        traj.points = [pt]
        
        self.arm_pub.publish(traj)
    
    def test_j4_compensation(self):
        """测试J4补偿"""
        if not self.mc:
            rospy.logerr("❌ Pro450未连接")
            return
        
        print("\n🧪 J4偏移补偿测试")
        print("="*40)
        
        # 测试角度
        test_cases = [
            {"name": "零位", "angles": [0, 0, 0, 0, 0, 0]},
            {"name": "J4=30°", "angles": [0, 0, 0, 30, 0, 0]},
            {"name": "J4=60°", "angles": [0, 0, 0, 60, 0, 0]},
            {"name": "J4=-30°", "angles": [0, 0, 0, -30, 0, 0]},
        ]
        
        for case in test_cases:
            print(f"\n📍 测试: {case['name']}")
            print(f"目标角度: {case['angles']}")
            
            # 1. 发送到Gazebo
            self.send_to_gazebo(case['angles'])
            time.sleep(3.0)
            
            # 2. 读取Gazebo实际角度
            gazebo_actual = [self.gazebo_angles.get(f"joint{i+1}", 0) for i in range(6)]
            print(f"Gazebo实际: {[round(a, 1) for a in gazebo_actual]}")
            
            # 3. 不补偿直接发送到真实机械臂
            print("  🔸 不补偿测试:")
            self.mc.send_angles(gazebo_actual, 50)
            time.sleep(3.0)
            real_no_comp = self.mc.get_angles()
            print(f"    真实机械臂: {[round(a, 1) for a in real_no_comp]}")
            print(f"    J4差异: {gazebo_actual[3] - real_no_comp[3]:.1f}°")
            
            # 4. 补偿后发送到真实机械臂
            print("  🔹 补偿测试 (J4-10°):")
            compensated = self.compensate_j4_angle(gazebo_actual, -10.0)
            self.mc.send_angles(compensated, 50)
            time.sleep(3.0)
            real_comp = self.mc.get_angles()
            print(f"    发送角度: {[round(a, 1) for a in compensated]}")
            print(f"    真实机械臂: {[round(a, 1) for a in real_comp]}")
            print(f"    J4差异: {gazebo_actual[3] - real_comp[3]:.1f}°")
            
            input("  ⏸️  按Enter继续...")
    
    def interactive_test(self):
        """交互式测试"""
        while not rospy.is_shutdown():
            print(f"\n🧪 J4偏移测试器")
            print("="*25)
            print("1. 连接Pro450")
            print("2. 运行补偿测试")
            print("3. 退出")
            
            try:
                choice = input("\n请选择 (1-3): ").strip()
                
                if choice == '1':
                    if self.connect_pro450():
                        print("✅ Pro450连接成功")
                    else:
                        print("❌ Pro450连接失败")
                elif choice == '2':
                    self.test_j4_compensation()
                elif choice == '3':
                    print("👋 退出测试器")
                    break
                else:
                    print("❌ 无效选择")
                    
            except KeyboardInterrupt:
                print("\n👋 退出测试器")
                break

def main():
    try:
        tester = J4OffsetTest()
        rospy.sleep(2.0)
        tester.interactive_test()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()