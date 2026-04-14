#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joint_monitor.py - MyCobot Pro 450 关节角度监控器
提供实时监控、数据记录和角度查询功能
"""

import rospy
import math
import time
import json
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointMonitor:
    def __init__(self):
        rospy.init_node('joint_monitor', anonymous=True)
        
        # 关节配置
        self.joint_names = [
            "joint1", "joint2", "joint3", 
            "joint4", "joint5", "joint6"
        ]
        self.gripper_joint = "gripper_controller"
        
        # 数据存储
        self.current_angles = {}
        self.angle_history = []
        self.last_update_time = None
        
        # 监控配置
        self.monitoring = True
        self.log_interval = 1.0  # 记录间隔(秒)
        self.last_log_time = 0
        
        # 订阅关节状态
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        
        # 发布器(可选，用于重新发布处理后的数据)
        self.angle_pub = rospy.Publisher("/mycobot/joint_angles", JointState, queue_size=10)
        
        rospy.loginfo("🤖 MyCobot Pro 450 关节监控器已启动")
        rospy.loginfo("📊 功能: 实时监控、数据记录、角度查询")
        
    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        current_time = time.time()
        
        # 解析关节角度
        arm_angles = {}
        gripper_angle = None
        
        for name, position in zip(msg.name, msg.position):
            angle_deg = math.degrees(position)
            
            if name in self.joint_names:
                arm_angles[name] = round(angle_deg, 2)
            elif name == self.gripper_joint:
                gripper_angle = round(angle_deg, 2)
        
        # 更新当前角度
        self.current_angles.update(arm_angles)
        if gripper_angle is not None:
            self.current_angles[self.gripper_joint] = gripper_angle
        
        self.last_update_time = current_time
        
        # 定期记录数据
        if current_time - self.last_log_time >= self.log_interval:
            self.log_angles()
            self.last_log_time = current_time
        
        # 发布处理后的角度数据
        self.publish_angles(msg.header)
    
    def log_angles(self):
        """记录角度数据到历史"""
        if not self.current_angles:
            return
        
        timestamp = time.time()
        angle_record = {
            'timestamp': timestamp,
            'time_str': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp)),
            'angles': self.current_angles.copy()
        }
        
        self.angle_history.append(angle_record)
        
        # 限制历史记录长度(保留最近100条)
        if len(self.angle_history) > 100:
            self.angle_history.pop(0)
        
        # 打印当前状态
        self.print_current_status()
    
    def print_current_status(self):
        """打印当前关节状态"""
        print(f"\n⏰ {time.strftime('%H:%M:%S')} - MyCobot Pro 450 关节状态:")
        print("-" * 60)
        
        # 打印臂关节
        print("🦾 机械臂关节:")
        for joint_name in self.joint_names:
            if joint_name in self.current_angles:
                angle = self.current_angles[joint_name]
                print(f"  {joint_name:8}: {angle:8.2f}°")
        
        # 打印夹爪
        if self.gripper_joint in self.current_angles:
            gripper_angle = self.current_angles[self.gripper_joint]
            print(f"🤏 夹爪:")
            print(f"  gripper : {gripper_angle:8.2f}°")
        
        print("-" * 60)
    
    def publish_angles(self, header):
        """发布角度数据"""
        if not self.current_angles:
            return
        
        # 创建JointState消息
        joint_msg = JointState()
        joint_msg.header = header
        joint_msg.header.stamp = rospy.Time.now()
        
        # 添加关节数据
        for joint_name in self.joint_names + [self.gripper_joint]:
            if joint_name in self.current_angles:
                joint_msg.name.append(joint_name)
                joint_msg.position.append(math.radians(self.current_angles[joint_name]))
                joint_msg.velocity.append(0.0)
                joint_msg.effort.append(0.0)
        
        # 发布消息
        self.angle_pub.publish(joint_msg)
    
    def get_joint_angle(self, joint_name):
        """获取指定关节的当前角度"""
        return self.current_angles.get(joint_name, None)
    
    def get_all_angles(self):
        """获取所有关节的当前角度"""
        return self.current_angles.copy()
    
    def get_arm_angles(self):
        """获取机械臂关节角度(不包括夹爪)"""
        arm_angles = {}
        for joint_name in self.joint_names:
            if joint_name in self.current_angles:
                arm_angles[joint_name] = self.current_angles[joint_name]
        return arm_angles
    
    def get_gripper_angle(self):
        """获取夹爪角度"""
        return self.current_angles.get(self.gripper_joint, None)
    
    def save_history_to_file(self, filename=None):
        """保存角度历史到文件"""
        if not filename:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filename = f"mycobot_angles_{timestamp}.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.angle_history, f, indent=2)
            rospy.loginfo(f"📁 角度历史已保存到: {filename}")
            return True
        except Exception as e:
            rospy.logerr(f"❌ 保存失败: {e}")
            return False
    
    def print_statistics(self):
        """打印统计信息"""
        if not self.angle_history:
            print("📊 暂无历史数据")
            return
        
        print(f"\n📊 统计信息:")
        print(f"  记录数量: {len(self.angle_history)}")
        print(f"  监控时长: {len(self.angle_history) * self.log_interval:.1f} 秒")
        
        if self.last_update_time:
            print(f"  最后更新: {time.strftime('%H:%M:%S', time.localtime(self.last_update_time))}")
    
    def interactive_mode(self):
        """交互模式"""
        print("\n🎮 进入交互模式 (输入 'help' 查看命令)")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'help':
                    print("📋 可用命令:")
                    print("  current  - 显示当前角度")
                    print("  joint <name> - 显示指定关节角度")
                    print("  arm      - 显示机械臂关节角度")
                    print("  gripper  - 显示夹爪角度")
                    print("  stats    - 显示统计信息")
                    print("  save     - 保存历史数据")
                    print("  quit     - 退出程序")
                
                elif cmd == 'current':
                    self.print_current_status()
                
                elif cmd.startswith('joint '):
                    joint_name = cmd.split(' ', 1)[1]
                    angle = self.get_joint_angle(joint_name)
                    if angle is not None:
                        print(f"🔧 {joint_name}: {angle:.2f}°")
                    else:
                        print(f"❌ 关节 '{joint_name}' 未找到")
                
                elif cmd == 'arm':
                    arm_angles = self.get_arm_angles()
                    print("🦾 机械臂关节角度:")
                    for name, angle in arm_angles.items():
                        print(f"  {name}: {angle:.2f}°")
                
                elif cmd == 'gripper':
                    gripper_angle = self.get_gripper_angle()
                    if gripper_angle is not None:
                        print(f"🤏 夹爪角度: {gripper_angle:.2f}°")
                    else:
                        print("❌ 夹爪角度未获取到")
                
                elif cmd == 'stats':
                    self.print_statistics()
                
                elif cmd == 'save':
                    self.save_history_to_file()
                
                elif cmd in ['quit', 'exit', 'q']:
                    print("👋 退出交互模式")
                    break
                
                else:
                    print("❓ 未知命令，输入 'help' 查看帮助")
                    
            except KeyboardInterrupt:
                print("\n👋 退出交互模式")
                break
            except EOFError:
                print("\n👋 退出交互模式")
                break

def main():
    try:
        # 创建监控器
        monitor = JointMonitor()
        
        # 等待一下让数据稳定
        rospy.sleep(2.0)
        
        print("\n🚀 监控器启动完成!")
        print("💡 选择运行模式:")
        print("  1. 自动监控模式 (持续显示角度)")
        print("  2. 交互查询模式 (手动查询)")
        
        choice = input("请选择 (1/2, 默认1): ").strip()
        
        if choice == '2':
            monitor.interactive_mode()
        else:
            print("🔄 自动监控模式启动，按 Ctrl+C 退出")
            rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        rospy.loginfo("👋 关节监控器已退出")

if __name__ == "__main__":
    main()