#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
real_cause_analyzer.py - 真实原因分析器
重新分析Joint2和Joint4与真实机械臂角度偏差的真正原因
"""

import rospy
import math
import time
from sensor_msgs.msg import JointState
from pymycobot import Pro450Client

class RealCauseAnalyzer:
    def __init__(self):
        rospy.init_node('real_cause_analyzer', anonymous=True)
        
        # 关节配置
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # 数据存储
        self.gazebo_angles = {}
        
        # 订阅Gazebo关节状态
        rospy.Subscriber("/joint_states", JointState, self.gazebo_callback)
        
        rospy.loginfo("🔍 真实原因分析器已启动")
        
    def gazebo_callback(self, msg):
        """处理Gazebo关节状态"""
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.gazebo_angles[name] = math.degrees(position)
    
    def analyze_possible_causes(self):
        """分析可能的真实原因"""
        print("\n🔍 Joint2和Joint4角度偏差的真实原因分析")
        print("="*60)
        
        print("❌ 排除的原因:")
        print("  - PID参数: 只影响Gazebo仿真控制，不影响角度读取")
        print("  - 控制器设置: 不会改变角度的对应关系")
        print("")
        
        print("🎯 可能的真实原因:")
        print("")
        
        print("1️⃣ **URDF模型与真实机械臂的DH参数差异**")
        print("   - Gazebo使用URDF定义的关节轴向和位置")
        print("   - 真实机械臂的实际DH参数可能与URDF不完全一致")
        print("   - 特别是Joint2和Joint4的轴向定义可能有偏差")
        print("")
        
        print("2️⃣ **关节零位定义不同**")
        print("   - Gazebo中的关节零位 ≠ 真实机械臂的关节零位")
        print("   - 可能存在固定的角度偏移")
        print("   - Joint2和Joint4的零位校准可能不准确")
        print("")
        
        print("3️⃣ **坐标系定义差异**")
        print("   - URDF中的关节坐标系方向")
        print("   - 真实机械臂固件中的坐标系方向")
        print("   - 可能存在轴向反转或偏移")
        print("")
        
        print("4️⃣ **机械装配误差**")
        print("   - 真实机械臂在装配时的机械误差")
        print("   - 关节编码器的安装位置偏差")
        print("   - 减速器的机械间隙")
        print("")
        
        print("5️⃣ **固件角度计算差异**")
        print("   - 真实机械臂固件的角度计算方法")
        print("   - 可能使用不同的运动学模型")
        print("   - 编码器读数到角度的转换公式")
        print("")
        
        # 分析URDF中的关节定义
        print("6️⃣ **URDF关节定义分析**")
        self.analyze_urdf_joints()
    
    def analyze_urdf_joints(self):
        """分析URDF中的关节定义"""
        print("\n📋 URDF关节定义详细分析:")
        
        # 从之前读取的URDF文件分析
        joint_info = {
            "joint1": {
                "axis": "(0, 0, 1)",  # Z轴
                "origin": "(-0.0045, 0, 0.155)",
                "limits": "±165°",
                "type": "基座旋转"
            },
            "joint2": {
                "axis": "(1, 0, 0)",  # X轴 ⚠️
                "origin": "(0.05, 0, 0.048)",
                "limits": "±120°",
                "type": "肩部俯仰"
            },
            "joint3": {
                "axis": "(1, 0, 0)",  # X轴
                "origin": "(-0.01, 0, 0.18)",
                "limits": "±158°",
                "type": "肘部俯仰"
            },
            "joint4": {
                "axis": "(1, 0, 0)",  # X轴 ⚠️
                "origin": "(0.0, 0, 0.1735)",
                "limits": "±165°",
                "type": "腕部俯仰"
            },
            "joint5": {
                "axis": "(0, 0, 1)",  # Z轴
                "origin": "(0.046, 0.0003, 0.05)",
                "limits": "±165°",
                "type": "腕部旋转"
            },
            "joint6": {
                "axis": "(1, 0, 0)",  # X轴
                "origin": "(0, 0.04, 0.0435)",
                "limits": "±175°",
                "type": "末端旋转"
            }
        }
        
        for joint, info in joint_info.items():
            marker = "🔴" if joint in ["joint2", "joint4"] else "🟢"
            print(f"  {marker} {joint}:")
            print(f"    轴向: {info['axis']}")
            print(f"    位置: {info['origin']}")
            print(f"    限制: {info['limits']}")
            print(f"    类型: {info['type']}")
            print("")
    
    def suggest_diagnostic_steps(self):
        """建议诊断步骤"""
        print("🔧 建议的诊断步骤:")
        print("")
        
        print("步骤1: 零位对比测试")
        print("  - 将Gazebo和真实机械臂都设置为零位 [0,0,0,0,0,0]")
        print("  - 对比两者的实际姿态是否一致")
        print("  - 如果不一致，说明零位定义有差异")
        print("")
        
        print("步骤2: 单关节测试")
        print("  - 逐个测试每个关节的90°位置")
        print("  - 观察哪些关节的实际姿态与Gazebo不符")
        print("  - 记录具体的角度偏差")
        print("")
        
        print("步骤3: 轴向验证")
        print("  - 手动旋转真实机械臂的Joint2和Joint4")
        print("  - 观察旋转轴向是否与URDF定义一致")
        print("  - 检查正方向定义是否相同")
        print("")
        
        print("步骤4: 角度映射测试")
        print("  - 测试多个角度点: -90°, -45°, 0°, 45°, 90°")
        print("  - 建立Gazebo角度 vs 真实角度的映射关系")
        print("  - 查看是否存在线性关系或固定偏移")
        print("")
        
        print("步骤5: 固件信息检查")
        print("  - 检查真实机械臂的固件版本")
        print("  - 查看是否有角度校准或DH参数设置")
        print("  - 确认编码器的零位设置")
    
    def create_calibration_test(self):
        """创建校准测试程序"""
        print("\n🧪 校准测试程序")
        print("="*40)
        
        test_angles = [
            [0, 0, 0, 0, 0, 0],      # 零位
            [0, 45, 0, 0, 0, 0],     # Joint2测试
            [0, 0, 0, 45, 0, 0],     # Joint4测试
            [0, -45, 0, 0, 0, 0],    # Joint2负角度
            [0, 0, 0, -45, 0, 0],    # Joint4负角度
            [0, 90, 0, 0, 0, 0],     # Joint2大角度
            [0, 0, 0, 90, 0, 0],     # Joint4大角度
        ]
        
        print("建议测试角度序列:")
        for i, angles in enumerate(test_angles):
            print(f"  测试{i+1}: {angles}")
        
        print("\n对于每个测试:")
        print("  1. 发送角度到Gazebo")
        print("  2. 读取Gazebo实际角度")
        print("  3. 发送相同角度到真实机械臂")
        print("  4. 读取真实机械臂角度")
        print("  5. 计算并记录偏差")
        print("  6. 拍照记录实际姿态对比")
    
    def analyze_joint_naming(self):
        """分析关节命名和编号"""
        print("\n🏷️ 关节命名和编号分析")
        print("="*40)
        
        print("可能的问题:")
        print("  - Gazebo中的joint1-6编号")
        print("  - 真实机械臂中的关节编号")
        print("  - 是否存在编号不对应的情况")
        print("")
        
        print("检查方法:")
        print("  1. 确认Gazebo中joint2对应真实机械臂的哪个物理关节")
        print("  2. 确认Gazebo中joint4对应真实机械臂的哪个物理关节")
        print("  3. 检查关节编号是否从0开始还是从1开始")
        print("  4. 验证关节顺序是否一致")
    
    def interactive_diagnosis(self):
        """交互式诊断"""
        while not rospy.is_shutdown():
            print(f"\n🔍 真实原因分析器")
            print("="*30)
            print("1. 分析可能原因")
            print("2. 建议诊断步骤")
            print("3. 校准测试程序")
            print("4. 关节命名分析")
            print("5. 退出")
            
            try:
                choice = input("\n请选择 (1-5): ").strip()
                
                if choice == '1':
                    self.analyze_possible_causes()
                elif choice == '2':
                    self.suggest_diagnostic_steps()
                elif choice == '3':
                    self.create_calibration_test()
                elif choice == '4':
                    self.analyze_joint_naming()
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
        analyzer = RealCauseAnalyzer()
        
        # 等待ROS数据稳定
        rospy.sleep(2.0)
        
        analyzer.interactive_diagnosis()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")

if __name__ == "__main__":
    main()