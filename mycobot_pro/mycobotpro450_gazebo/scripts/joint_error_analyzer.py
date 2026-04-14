#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joint_error_analyzer.py - 关节误差分析工具
专门分析第二关节和第四关节误差的原因
"""

import rospy
import math
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot import Pro450Client

class JointErrorAnalyzer:
    def __init__(self):
        rospy.init_node('joint_error_analyzer', anonymous=True)
        
        # 关节配置
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # 数据存储
        self.gazebo_angles = {}
        self.real_angles = {}
        self.error_history = []
        
        # Pro450连接
        self.mc = None
        self.pro450_connected = False
        
        # 订阅Gazebo关节状态
        rospy.Subscriber("/joint_states", JointState, self.gazebo_callback)
        
        # 发布器
        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        
        rospy.loginfo("🔍 关节误差分析器已启动")
        
    def gazebo_callback(self, msg):
        """处理Gazebo关节状态"""
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.gazebo_angles[name] = math.degrees(position)
    
    def connect_pro450(self):
        """连接Pro450机械臂"""
        try:
            rospy.loginfo("🔌 正在连接Pro450...")
            self.mc = Pro450Client("192.168.0.232", 4500)
            time.sleep(1.0)
            
            # 上电和校准
            self.mc.power_on()
            time.sleep(1.0)
            self.mc.set_servo_calibration(6)
            time.sleep(0.5)
            
            # 测试连接
            test_angles = self.mc.get_angles()
            if test_angles:
                self.pro450_connected = True
                rospy.loginfo("✅ Pro450连接成功")
                return True
            else:
                rospy.logerr("❌ 无法获取Pro450角度")
                return False
                
        except Exception as e:
            rospy.logerr(f"❌ Pro450连接失败: {e}")
            return False
    
    def get_real_angles(self):
        """获取真实机械臂角度"""
        if not self.pro450_connected:
            return None
        
        try:
            angles = self.mc.get_angles()
            if angles and len(angles) == 6:
                angle_dict = {}
                for i, angle in enumerate(angles):
                    angle_dict[f"joint{i+1}"] = angle
                return angle_dict
        except Exception as e:
            rospy.logwarn(f"获取真实角度失败: {e}")
        
        return None
    
    def send_angles_to_real_robot(self, angles_dict):
        """发送角度到真实机械臂"""
        if not self.pro450_connected:
            return False
        
        try:
            angles_list = [angles_dict[f"joint{i+1}"] for i in range(6)]
            self.mc.send_angles(angles_list, 50)
            return True
        except Exception as e:
            rospy.logwarn(f"发送角度失败: {e}")
            return False
    
    def send_angles_to_gazebo(self, angles_dict):
        """发送角度到Gazebo"""
        try:
            traj = JointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.joint_names = self.joint_names
            
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(angles_dict[name]) for name in self.joint_names]
            pt.velocities = [0.0] * 6
            pt.time_from_start = rospy.Duration(2.0)
            traj.points = [pt]
            
            self.arm_pub.publish(traj)
            return True
        except Exception as e:
            rospy.logwarn(f"发送Gazebo角度失败: {e}")
            return False
    
    def analyze_joint_differences(self):
        """分析关节差异的可能原因"""
        print("\n🔍 关节差异分析")
        print("="*60)
        
        # 1. URDF关节轴向分析
        print("1️⃣ URDF关节轴向配置:")
        joint_axes = {
            "joint1": "Z轴 (0 0 1) - 基座旋转",
            "joint2": "X轴 (1 0 0) - 肩部俯仰 ⚠️",
            "joint3": "X轴 (1 0 0) - 肘部俯仰",
            "joint4": "X轴 (1 0 0) - 腕部俯仰 ⚠️",
            "joint5": "Z轴 (0 0 1) - 腕部旋转",
            "joint6": "X轴 (1 0 0) - 末端旋转"
        }
        
        for joint, desc in joint_axes.items():
            marker = "🔴" if joint in ["joint2", "joint4"] else "🟢"
            print(f"  {marker} {joint}: {desc}")
        
        # 2. PID参数分析
        print(f"\n2️⃣ PID控制器参数差异:")
        pid_params = {
            "joint1": {"p": 10000.0, "i": 100.0, "d": 100.0},
            "joint2": {"p": 10000.0, "i": 100.0, "d": 100.0},
            "joint3": {"p": 10000.0, "i": 100.0, "d": 100.0},
            "joint4": {"p": 5000.0, "i": 50.0, "d": 50.0},  # 不同的PID参数
            "joint5": {"p": 5000.0, "i": 50.0, "d": 50.0},
            "joint6": {"p": 5000.0, "i": 50.0, "d": 50.0}
        }
        
        for joint, params in pid_params.items():
            marker = "🔴" if joint in ["joint2", "joint4"] else "🟢"
            print(f"  {marker} {joint}: P={params['p']}, I={params['i']}, D={params['d']}")
        
        # 3. 关节限制分析
        print(f"\n3️⃣ 关节角度限制 (弧度):")
        joint_limits = {
            "joint1": (-2.8274, 2.8274),  # ±162°
            "joint2": (-2.1816, 2.1816),  # ±125°
            "joint3": (-2.6878, 2.6878),  # ±154°
            "joint4": (-2.8274, 2.8274),  # ±162°
            "joint5": (-2.8274, 2.8274),  # ±162°
            "joint6": (-2.8797, 2.8797)   # ±165°
        }
        
        for joint, (lower, upper) in joint_limits.items():
            lower_deg = math.degrees(lower)
            upper_deg = math.degrees(upper)
            marker = "🔴" if joint == "joint2" else "🟢"
            print(f"  {marker} {joint}: {lower_deg:.1f}° 到 {upper_deg:.1f}°")
        
        # 4. 可能的误差原因
        print(f"\n4️⃣ 可能的误差原因:")
        print("🔴 joint2 (第二关节):")
        print("   - 角度限制较小 (±120° vs 其他关节的±165°)")
        print("   - X轴旋转，可能存在重力影响")
        print("   - 肩部关节，承受较大负载")
        print("   - 可能存在机械间隙或齿轮减速比差异")
        
        print("\n🔴 joint4 (第四关节):")
        print("   - PID参数较低 (P=5000 vs joint2的P=10000)")
        print("   - 腕部关节，位置较远，累积误差影响")
        print("   - X轴旋转，可能受前面关节影响")
        print("   - 较小的惯性，控制精度可能不同")
    
    def run_error_test(self):
        """运行误差测试"""
        if not self.pro450_connected:
            rospy.logerr("❌ Pro450未连接，无法进行测试")
            return
        
        print(f"\n🧪 开始误差测试")
        print("="*50)
        
        # 测试角度序列
        test_angles_list = [
            {"name": "零位", "angles": [0, 0, 0, 0, 0, 0]},
            {"name": "joint2测试", "angles": [0, 45, 0, 0, 0, 0]},
            {"name": "joint4测试", "angles": [0, 0, 0, 45, 0, 0]},
            {"name": "joint2+4测试", "angles": [0, 30, 0, 30, 0, 0]},
            {"name": "全关节测试", "angles": [30, 30, 30, 30, 30, 30]}
        ]
        
        for test_case in test_angles_list:
            print(f"\n📍 测试: {test_case['name']}")
            print(f"目标角度: {test_case['angles']}")
            
            # 构建角度字典
            target_angles = {}
            for i, angle in enumerate(test_case['angles']):
                target_angles[f"joint{i+1}"] = angle
            
            # 发送到Gazebo
            print("  📤 发送到Gazebo...")
            if self.send_angles_to_gazebo(target_angles):
                time.sleep(3.0)  # 等待Gazebo到达位置
                
                # 读取Gazebo实际角度
                gazebo_actual = self.gazebo_angles.copy()
                print(f"  📥 Gazebo实际: {[round(gazebo_actual.get(f'joint{i+1}', 0), 2) for i in range(6)]}")
                
                # 发送到真实机械臂
                print("  📤 发送到真实机械臂...")
                if self.send_angles_to_real_robot(gazebo_actual):
                    time.sleep(4.0)  # 等待真实机械臂到达位置
                    
                    # 读取真实机械臂角度
                    real_actual = self.get_real_angles()
                    if real_actual:
                        print(f"  📥 真实机械臂: {[round(real_actual.get(f'joint{i+1}', 0), 2) for i in range(6)]}")
                        
                        # 计算误差
                        errors = {}
                        for joint in self.joint_names:
                            gazebo_angle = gazebo_actual.get(joint, 0)
                            real_angle = real_actual.get(joint, 0)
                            error = abs(gazebo_angle - real_angle)
                            errors[joint] = error
                        
                        print(f"  📊 误差分析:")
                        for joint in self.joint_names:
                            error = errors[joint]
                            marker = "🔴" if error > 2.0 else "🟡" if error > 1.0 else "🟢"
                            print(f"    {marker} {joint}: {error:.2f}°")
                        
                        # 记录误差历史
                        self.error_history.append({
                            'test': test_case['name'],
                            'target': test_case['angles'],
                            'gazebo': [gazebo_actual.get(f'joint{i+1}', 0) for i in range(6)],
                            'real': [real_actual.get(f'joint{i+1}', 0) for i in range(6)],
                            'errors': [errors.get(f'joint{i+1}', 0) for i in range(6)]
                        })
                    
                    input("  ⏸️  按Enter继续下一个测试...")
    
    def print_error_summary(self):
        """打印误差总结"""
        if not self.error_history:
            print("❌ 没有测试数据")
            return
        
        print(f"\n📊 误差总结报告")
        print("="*60)
        
        # 计算平均误差
        joint_errors = {f"joint{i+1}": [] for i in range(6)}
        
        for record in self.error_history:
            for i, error in enumerate(record['errors']):
                joint_errors[f"joint{i+1}"].append(error)
        
        print("平均误差:")
        for joint in self.joint_names:
            errors = joint_errors[joint]
            if errors:
                avg_error = sum(errors) / len(errors)
                max_error = max(errors)
                marker = "🔴" if avg_error > 2.0 else "🟡" if avg_error > 1.0 else "🟢"
                print(f"  {marker} {joint}: 平均={avg_error:.2f}°, 最大={max_error:.2f}°")
        
        # 问题关节分析
        problem_joints = []
        for joint in self.joint_names:
            errors = joint_errors[joint]
            if errors and sum(errors) / len(errors) > 2.0:
                problem_joints.append(joint)
        
        if problem_joints:
            print(f"\n🚨 问题关节: {', '.join(problem_joints)}")
            print("建议检查:")
            for joint in problem_joints:
                if joint == "joint2":
                    print("  - joint2: 检查肩部机械间隙、重力补偿、PID调优")
                elif joint == "joint4":
                    print("  - joint4: 检查PID参数、机械精度、累积误差")
    
    def interactive_mode(self):
        """交互模式"""
        while not rospy.is_shutdown():
            print(f"\n🎮 关节误差分析器 - 交互模式")
            print("="*40)
            print("1. 连接Pro450机械臂")
            print("2. 分析关节差异原因")
            print("3. 运行误差测试")
            print("4. 查看误差总结")
            print("5. 退出")
            
            try:
                choice = input("\n请选择 (1-5): ").strip()
                
                if choice == '1':
                    self.connect_pro450()
                elif choice == '2':
                    self.analyze_joint_differences()
                elif choice == '3':
                    self.run_error_test()
                elif choice == '4':
                    self.print_error_summary()
                elif choice == '5':
                    print("👋 退出分析器")
                    break
                else:
                    print("❌ 无效选择")
                    
            except KeyboardInterrupt:
                print("\n👋 退出分析器")
                break

def main():
    try:
        analyzer = JointErrorAnalyzer()
        
        # 等待ROS数据稳定
        rospy.sleep(2.0)
        
        analyzer.interactive_mode()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")

if __name__ == "__main__":
    main()