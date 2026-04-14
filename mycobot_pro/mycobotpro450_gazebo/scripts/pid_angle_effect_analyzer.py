#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_angle_effect_analyzer.py - PID参数对角度影响的分析器
分析为什么调整PID参数会影响与真实机械臂的角度偏差
"""

import rospy
import math
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PIDAngleEffectAnalyzer:
    def __init__(self):
        rospy.init_node('pid_angle_effect_analyzer', anonymous=True)
        
        # 关节名称
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Gazebo角度
        self.gazebo_angles = {}
        
        # 订阅Gazebo状态
        rospy.Subscriber("/joint_states", JointState, self.gazebo_callback)
        
        # 发布器
        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        
        rospy.loginfo("🔍 PID角度影响分析器已启动")
    
    def gazebo_callback(self, msg):
        """处理Gazebo关节状态"""
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.gazebo_angles[name] = math.degrees(position)
    
    def analyze_pid_angle_relationship(self):
        """分析PID参数与角度偏差的关系"""
        print("\n🔍 PID参数影响角度偏差的可能原因分析")
        print("="*60)
        
        print("🎯 **你的发现很重要！**")
        print("J2的P从10000改为5000后，角度偏差消失了")
        print("")
        
        print("🤔 **可能的原因分析：**")
        print("")
        
        print("1️⃣ **稳态误差和控制精度**")
        print("   - 高P值(10000)可能导致过度控制")
        print("   - 系统可能在目标位置附近震荡")
        print("   - 最终稳定位置可能偏离目标值")
        print("   - 降低P值(5000)让系统更平稳地到达目标")
        print("")
        
        print("2️⃣ **重力和负载补偿**")
        print("   - J2是肩部关节，承受最大重力负载")
        print("   - 高P值可能无法正确处理重力影响")
        print("   - 导致系统在重力作用下的平衡点偏移")
        print("   - 适中的P值可能更好地平衡控制力和重力")
        print("")
        
        print("3️⃣ **控制器饱和效应**")
        print("   - P值过高可能导致控制输出饱和")
        print("   - 饱和状态下系统无法精确控制")
        print("   - 降低P值避免饱和，提高控制精度")
        print("")
        
        print("4️⃣ **动力学模型差异**")
        print("   - Gazebo的动力学模型可能与真实机械臂不完全匹配")
        print("   - 不同的PID参数可能补偿了这种差异")
        print("   - 特定的P值可能更接近真实系统的响应特性")
        print("")
        
        print("5️⃣ **积分饱和和稳态误差**")
        print("   - 高P值配合I值可能导致积分饱和")
        print("   - 积分饱和会产生稳态偏差")
        print("   - 降低P值可能缓解积分饱和问题")
    
    def suggest_j4_pid_adjustment(self):
        """建议J4的PID调整"""
        print("\n🛠️ **基于J2成功经验的J4调整建议**")
        print("="*50)
        
        print("📊 **当前PID配置：**")
        print("   J2: P=10000 → P=5000 ✅ (已修复)")
        print("   J4: P=5000 (当前值)")
        print("")
        
        print("🎯 **J4调整策略：**")
        print("")
        
        print("策略1: 进一步降低J4的P值")
        print("   - 尝试 P=3000 或 P=2500")
        print("   - 观察角度偏差是否减小")
        print("")
        
        print("策略2: 调整I和D值")
        print("   - 当前: I=50, D=50")
        print("   - 尝试: I=30, D=30 (与P值成比例降低)")
        print("")
        
        print("策略3: 参考J2的比例关系")
        print("   - J2成功配置: P=5000, I=100, D=100")
        print("   - J4可以尝试: P=2500, I=50, D=50")
        print("   - 或者: P=3000, I=60, D=60")
        print("")
        
        print("🧪 **建议的测试步骤：**")
        print("1. 备份当前配置")
        print("2. 逐步降低J4的P值: 5000 → 4000 → 3000 → 2500")
        print("3. 每次调整后测试角度偏差")
        print("4. 找到最佳的P值")
        print("5. 微调I和D值优化性能")
    
    def create_pid_test_configs(self):
        """创建PID测试配置"""
        print("\n📋 **J4 PID测试配置建议**")
        print("="*40)
        
        configs = [
            {"name": "当前配置", "p": 5000, "i": 50, "d": 50},
            {"name": "配置1", "p": 4000, "i": 40, "d": 40},
            {"name": "配置2", "p": 3000, "i": 30, "d": 30},
            {"name": "配置3", "p": 2500, "i": 25, "d": 25},
            {"name": "配置4", "p": 2000, "i": 20, "d": 20},
            {"name": "参考J2", "p": 5000, "i": 100, "d": 100},
        ]
        
        print("建议按顺序测试以下配置：")
        print("")
        
        for i, config in enumerate(configs):
            print(f"{i+1}. {config['name']}:")
            print(f"   joint4: {{p: {config['p']}, i: {config['i']}, d: {config['d']}}}")
            print("")
        
        print("🔧 **修改方法：**")
        print("编辑文件: mycobotpro450_gazebo/config/ros_controllers.yaml")
        print("")
        print("找到这一段：")
        print("```yaml")
        print("gazebo_ros_control/pid_gains:")
        print("  joint4: {p: 5000.0, i: 50.0, d: 50.0}")
        print("```")
        print("")
        print("修改为测试配置，然后重启Gazebo")
    
    def analyze_why_pid_affects_angles(self):
        """分析为什么PID会影响角度"""
        print("\n🧠 **深度分析：为什么PID参数会影响角度偏差**")
        print("="*55)
        
        print("🔬 **理论分析：**")
        print("")
        
        print("1. **控制系统的稳态特性**")
        print("   - PID控制器的目标是让系统输出跟踪参考输入")
        print("   - 但在有干扰(重力、摩擦)的情况下，不同的PID参数")
        print("     会导致不同的稳态误差")
        print("   - 高P值可能导致系统在干扰下的平衡点偏移")
        print("")
        
        print("2. **Gazebo仿真的物理特性**")
        print("   - Gazebo模拟了重力、惯性、摩擦等物理效应")
        print("   - 不同的PID参数会影响系统如何响应这些物理效应")
        print("   - 可能存在一个'最佳'PID参数使仿真更接近真实")
        print("")
        
        print("3. **数值积分和离散化误差**")
        print("   - Gazebo使用数值方法求解动力学方程")
        print("   - 不同的控制增益可能放大或减小数值误差")
        print("   - 导致长期积累的角度偏差")
        print("")
        
        print("4. **控制器与物理引擎的交互**")
        print("   - PID控制器输出力矩到Gazebo物理引擎")
        print("   - 物理引擎计算关节运动并反馈位置")
        print("   - 这个闭环系统的特性受PID参数强烈影响")
        print("")
        
        print("🎯 **实际意义：**")
        print("   你的发现说明Gazebo仿真需要特定的PID参数")
        print("   才能准确模拟真实机械臂的行为特性！")
    
    def interactive_analysis(self):
        """交互式分析"""
        while not rospy.is_shutdown():
            print(f"\n🔍 PID角度影响分析器")
            print("="*30)
            print("1. 分析PID与角度关系")
            print("2. J4 PID调整建议")
            print("3. 创建测试配置")
            print("4. 深度理论分析")
            print("5. 退出")
            
            try:
                choice = input("\n请选择 (1-5): ").strip()
                
                if choice == '1':
                    self.analyze_pid_angle_relationship()
                elif choice == '2':
                    self.suggest_j4_pid_adjustment()
                elif choice == '3':
                    self.create_pid_test_configs()
                elif choice == '4':
                    self.analyze_why_pid_affects_angles()
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
        analyzer = PIDAngleEffectAnalyzer()
        rospy.sleep(2.0)
        analyzer.interactive_analysis()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()