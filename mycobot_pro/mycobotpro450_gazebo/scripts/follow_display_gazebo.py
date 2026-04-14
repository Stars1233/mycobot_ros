#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
follow_display_gazebo.py - MyCobot Pro 450 版本
功能：读取真实 Pro450 机械臂的关节角度，同步显示到 Gazebo 仿真中
用途：手动摆动真实机械臂时，Gazebo 中的虚拟机械臂会跟随移动

使用方法：
1. 确保真实机械臂已连接到网络 (默认 192.168.0.232:4500)
2. 启动 Gazebo 仿真
3. 运行此脚本
4. 手动摆动真实机械臂，观察 Gazebo 中的同步效果
"""

import math
import time
import threading
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot import Pro450Client

# 全局变量
mc = None
pub_arm_command = None
pub_gripper_command = None

# 上一次有效值
last_valid_angles = None
last_mapped_gripper = None

# Pro450 连接参数
PRO450_IP = "192.168.0.232"
PRO450_PORT = 4500

# 夹爪映射常量
GRIPPER_MIN_ANGLE = 0      # Pro450夹爪最小角度
GRIPPER_MAX_ANGLE = 100    # Pro450夹爪最大角度
GAZEBO_MIN_POSITION = 0 # Gazebo夹爪最小位置
GAZEBO_MAX_POSITION = 57.3   # Gazebo夹爪最大位置

# 夹爪ID - Pro力控夹爪
GRIPPER_ID = 14

# Pro450 关节名称（与URDF一致）
ARM_JOINT_NAMES = [
    "joint1",
    "joint2", 
    "joint3",
    "joint4",
    "joint5",
    "joint6"
]
GRIPPER_JOINT_NAMES = ["gripper_controller"]

# 同步频率
SYNC_RATE = 20  # Hz

# 角度缩放系数 (真实角度 * 系数 = Gazebo角度)
# 系数 > 1: Gazebo转得更多 (用于Gazebo比实际小的情况)
# 系数 < 1: Gazebo转得更少 (用于Gazebo比实际大的情况)
# 计算方法: 如果真实60°时Gazebo显示68°，则系数 = 60/68 ≈ 0.88
#          如果真实60°时Gazebo显示45°，则系数 = 60/45 ≈ 1.33
ANGLE_SCALE = [
    1.0,    # joint1 - 无缩放
    0.85,   # joint2 - Gazebo比实际大8°左右，需要缩小
    1.0,    # joint3 - Gazebo比实际小很多，重点放大
    1.35,   # joint4 - Gazebo比实际小15°左右，需要放大
    0.95,   # joint5 - Gazebo比实际小15°左右，需要放大
    1.0,    # joint6 - 无缩放
]

# 角度偏移 (在缩放后再加上偏移，单位：度)
# 零位正确时偏移应该为0
ANGLE_OFFSET = [
    0,  # joint1
    0,  # joint2
    0,  # joint3
    0,  # joint4
    0,  # joint5
    0,  # joint6
]

def is_valid_gripper_angle(angle):
    """检查夹爪角度是否有效"""
    if angle is None:
        return False
    if isinstance(angle, (int, float)):
        # 检查是否在合理范围内
        if 0 <= angle <= 100:
            return True
        # 排除异常值
        if angle == 65535 or angle == -1 or angle < -100 or angle > 1000:
            return False
    return False

def is_valid_angles(angles):
    """检查关节角度是否有效"""
    if angles is None:
        return False
    if not isinstance(angles, (list, tuple)):
        return False
    if len(angles) != 6:
        return False
    # 检查每个角度是否在合理范围内
    for a in angles:
        if not isinstance(a, (int, float)):
            return False
        if a < -180 or a > 180:
            return False
    return True

def initialize_gripper():
    """初始化Pro力控夹爪"""
    global mc
    
    try:
        rospy.loginfo("[夹爪] 尝试初始化Pro力控夹爪...")
        
        # 检查夹爪是否存在
        version = mc.get_pro_gripper(1, GRIPPER_ID)
        if version == -1:
            rospy.logwarn("[夹爪] 未检测到Pro力控夹爪")
            return False
        
        rospy.loginfo(f"[夹爪] ✅ Pro力控夹爪检测成功，版本: {version}")
        
        # 尝试获取当前角度
        try:
            current_angle = mc.get_pro_gripper_angle(GRIPPER_ID)
            rospy.loginfo(f"[夹爪] 当前夹爪角度: {current_angle}°")
        except Exception as e:
            rospy.logwarn(f"[夹爪] 无法获取夹爪角度: {e}")
        
        return True
        
    except Exception as e:
        rospy.logwarn(f"[夹爪] 初始化失败: {e}")
        return False

def sync_to_gazebo():
    """同步真实机械臂状态到Gazebo"""
    global last_valid_angles, last_mapped_gripper
    
    rate = rospy.Rate(SYNC_RATE)
    
    while not rospy.is_shutdown():
        try:
            # 获取真实机械臂关节角度
            angles = None
            for retry in range(3):
                try:
                    angles = mc.get_angles()
                    if is_valid_angles(angles):
                        last_valid_angles = angles[:]
                        break
                    time.sleep(0.02)
                except Exception as e:
                    time.sleep(0.02)
                    continue
            
            # 如果获取失败，使用上次有效值
            if not is_valid_angles(angles):
                if last_valid_angles is not None:
                    angles = last_valid_angles
                    rospy.logwarn_throttle(5, "[同步] 使用上次有效关节角度")
                else:
                    rospy.logwarn_throttle(2, "[同步] 无法获取关节角度，跳过此次更新")
                    rate.sleep()
                    continue
            
            # 获取夹爪角度
            gripper_angle = None
            valid_gripper_found = False
            
            for retry in range(3):
                try:
                    raw_angle = mc.get_pro_gripper_angle(GRIPPER_ID)
                    if is_valid_gripper_angle(raw_angle):
                        gripper_angle = raw_angle
                        valid_gripper_found = True
                        break
                    time.sleep(0.02)
                except:
                    time.sleep(0.02)
                    continue
            
            # 应用角度缩放和偏移补偿
            # 公式: Gazebo角度 = 真实角度 * 缩放系数 + 偏移
            compensated_angles = [angles[i] * ANGLE_SCALE[i] + ANGLE_OFFSET[i] for i in range(6)]
            
            # 发布机械臂轨迹到Gazebo
            arm_angles_rad = [math.radians(a) for a in compensated_angles]
            
            arm_traj = JointTrajectory()
            arm_traj.header.stamp = rospy.Time.now()
            arm_traj.joint_names = ARM_JOINT_NAMES
            
            pt = JointTrajectoryPoint()
            pt.positions = arm_angles_rad
            pt.velocities = [0.0] * 6
            pt.accelerations = [0.0] * 6
            pt.time_from_start = rospy.Duration(0.1)
            arm_traj.points.append(pt)
            
            pub_arm_command.publish(arm_traj)
            
            # 处理夹爪角度映射
            if valid_gripper_found and is_valid_gripper_angle(gripper_angle):
                # 正常映射: Pro450角度(0-100) -> Gazebo角度(-60到60)
                mapped_gripper = ((gripper_angle - GRIPPER_MIN_ANGLE) /
                                 (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE)) * \
                                (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
                
                # 限制范围
                mapped_gripper = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped_gripper))
                last_mapped_gripper = mapped_gripper
                
                rospy.logdebug(f"[夹爪] Pro450={gripper_angle}° -> Gazebo={mapped_gripper:.1f}°")
            else:
                # 使用上次有效值或默认值
                if last_mapped_gripper is not None:
                    mapped_gripper = last_mapped_gripper
                else:
                    mapped_gripper = GAZEBO_MIN_POSITION  # 默认关闭状态
            
            # 发布夹爪轨迹到Gazebo
            gripper_traj = JointTrajectory()
            gripper_traj.header.stamp = rospy.Time.now()
            gripper_traj.joint_names = GRIPPER_JOINT_NAMES
            
            gp = JointTrajectoryPoint()
            gp.positions = [math.radians(mapped_gripper)]
            gp.velocities = [0.0]
            gp.accelerations = [0.0]
            gp.time_from_start = rospy.Duration(0.1)
            gripper_traj.points.append(gp)
            
            pub_gripper_command.publish(gripper_traj)
            
            rospy.logdebug(f"[同步] 原始角度: {[round(a,1) for a in angles]}, 补偿后: {[round(a,1) for a in compensated_angles]}, 夹爪: {mapped_gripper:.1f}°")
            
        except Exception as e:
            rospy.logerr_throttle(5, f"[同步] 同步错误: {e}")
        
        rate.sleep()


def initialize_pro450():
    """初始化Pro450网络连接"""
    global mc
    
    try:
        rospy.loginfo(f"[Pro450] 正在连接 Pro450 @ {PRO450_IP}:{PRO450_PORT}...")
        mc = Pro450Client(PRO450_IP, PRO450_PORT)
        time.sleep(1.0)
        
        # 上电
        rospy.loginfo("[Pro450] ⚡ 机械臂上电中...")
        mc.power_on()
        time.sleep(1.0)
        rospy.loginfo("[Pro450] ✅ 机械臂已上电")
        
        # 设置舵机校准
        rospy.loginfo("[Pro450] 🔧设置舵机校准...")
        mc.set_servo_calibration(6)
        time.sleep(0.5)
        
        # 测试连接
        current_angles = mc.get_angles()
        rospy.loginfo(f"[Pro450] ✅ Pro450连接成功!")
        rospy.loginfo(f"[Pro450] 当前角度: {current_angles}")
        
        # 初始化夹爪
        gripper_ok = initialize_gripper()
        if gripper_ok:
            rospy.loginfo("[Pro450] ✅ 夹爪初始化成功")
        else:
            rospy.logwarn("[Pro450] ⚠️  夹爪未检测到或初始化失败")
        
        # 释放舵机，允许手动摆动
        rospy.loginfo("[Pro450] 🔓 释放所有舵机，可以手动摆动机械臂...")
        mc.release_all_servos()
        mc.set_pro_gripper_enabled(0,14)
        time.sleep(0.5)
        
        return True
        
    except Exception as e:
        rospy.logerr(f"[Pro450] ❌ Pro450初始化失败: {e}")
        rospy.logerr("[Pro450] 请检查:")
        rospy.logerr("[Pro450] 1. 机械臂是否正确连接到网络")
        rospy.logerr(f"[Pro450] 2. IP地址 {PRO450_IP} 是否正确")
        rospy.logerr("[Pro450] 3. 机械臂是否已开机并正常工作")
        rospy.logerr("[Pro450] 4. 防火墙是否阻止了连接")
        return False

def main():
    """主函数"""
    global mc, pub_arm_command, pub_gripper_command
    
    rospy.init_node("pro450_gazebo_sync", anonymous=True)
    
    print("\n" + "="*60)
    print("🤖 MyCobot Pro 450 Gazebo 同步控制器")
    print("="*60)
    print("功能: 读取真实机械臂状态，同步到 Gazebo 仿真")
    print("用法: 手动摆动真实机械臂，Gazebo 中的虚拟机械臂会跟随移动")
    print("="*60 + "\n")
    
    # 初始化 Pro450 连接
    if not initialize_pro450():
        rospy.logerr("[main] ❌ Pro450初始化失败，退出")
        return
    
    # 初始化ROS发布者
    pub_arm_command = rospy.Publisher("/arm_controller/command",
                                      JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command",
                                          JointTrajectory, queue_size=10)
    rospy.loginfo("[main] ROS发布器初始化完成")
    
    rospy.loginfo("[main] Pro450 Gazebo同步控制节点已启动")
    rospy.loginfo("[main] 开始同步真实机械臂状态到Gazebo仿真...")
    rospy.loginfo(f"[main] 夹爪映射范围: Pro450(0-100) -> Gazebo({GAZEBO_MIN_POSITION}°到{GAZEBO_MAX_POSITION}°)")
    rospy.loginfo("[main] ✅ 舵机保持释放状态，您可以手动摆动机械臂")
    rospy.loginfo("[main] 💡 按 Ctrl+C 安全退出")
    
    # 注册退出处理函数
    def cleanup():
        if mc is not None:
            try:
                rospy.loginfo("[main] 程序退出，确保舵机释放...")
                mc.release_all_servos()
                rospy.loginfo("[main] ✅ 已释放所有舵机")
            except:
                pass
    
    rospy.on_shutdown(cleanup)
    
    # 启动同步线程
    sync_thread = threading.Thread(target=sync_to_gazebo, daemon=True)
    sync_thread.start()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[main] 🛑 接收到退出信号，正在关闭...")
    finally:
        cleanup()
        rospy.loginfo("[main] 👋 程序已安全退出")

if __name__ == "__main__":
    main()

