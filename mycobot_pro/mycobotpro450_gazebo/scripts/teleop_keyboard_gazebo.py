#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MyCobot Pro 450 键盘控制器
功能：键盘同时控制 Gazebo 仿真和真实 Pro450 机械臂
包括6个关节和夹爪的同步控制
"""

import math
import rospy
import time
import sys
import select
import termios
import tty
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot import Pro450Client

# ========================= 全局变量 =========================
mc = None  # Pro450 客户端
pub_arm_command = None
pub_gripper_command = None
home_pose = [0, 0, 0, 0, 0, 0]

# 命令执行控制
executor_running = True

# Pro450 连接参数
PRO450_IP = "192.168.0.232"
PRO450_PORT = 4500

# 关节和夹爪配置 (MyCobot Pro 450)
ARM_JOINTS = [
    "joint1",
    "joint2", 
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]
GRIPPER_JOINT = "gripper_controller"

# 安全角度限制 (度) - 根据更新后的Pro450 URDF定义 (2024.12)
JOINT_LIMITS = [
    (-162, 162),  # joint1 - 底座旋转 (±2.8274 rad)
    (-125, 125),  # joint2 - 大臂俯仰 (±2.1816 rad)
    (-154, 154),  # joint3 - 小臂俯仰 (±2.6878 rad)
    (-162, 162),  # joint4 - 腕部俯仰 (±2.8274 rad)
    (-162, 162),  # joint5 - 腕部旋转 (±2.8274 rad)
    (-165, 165),  # joint6 - 末端旋转 (±2.8797 rad)
]

# 夹爪配置 - Pro力控夹爪
GRIPPER_ID = 14
GRIPPER_MIN_ANGLE = 0      
GRIPPER_MAX_ANGLE = 100    
GAZEBO_MIN_POSITION = 0  
GAZEBO_MAX_POSITION = 57.3   

# 控制参数
ANGLE_STEP = 5.0            # 每次按键的角度步长
FAST_STEP = 15.0            # 快速移动步长
ROBOT_SPEED = 50            # 真实机械臂运动速度

# 当前状态
current_angles = [0, 0, 0, 0, 0, 0]
current_gripper_angle = 50  # 夹爪默认中间位置 (0-100)

# ========================= 安全检查 =========================
def clamp_angles(angles):
    """将角度限制在安全范围内"""
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angle = max(min_limit, min(max_limit, angle))
        clamped_angles.append(clamped_angle)
    return clamped_angles

# ========================= Pro450 初始化 =========================
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

def initialize_pro450():
    """初始化Pro450网络连接"""
    global mc, current_angles, current_gripper_angle
    
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
        rospy.loginfo("[Pro450] 🔧 设置舵机校准...")
        mc.set_servo_calibration(6)
        time.sleep(0.5)
        
        # 测试连接并获取当前角度
        angles = mc.get_angles()
        if angles and len(angles) == 6:
            current_angles = angles[:]
            rospy.loginfo(f"[Pro450] ✅ Pro450连接成功!")
            rospy.loginfo(f"[Pro450] 当前角度: {[round(a,1) for a in current_angles]}")
        else:
            rospy.logwarn("[Pro450] 无法获取当前角度，使用默认值")
        
        # 初始化夹爪
        gripper_ok = initialize_gripper()
        if gripper_ok:
            rospy.loginfo("[Pro450] ✅ 夹爪初始化成功")
            # 获取当前夹爪角度
            try:
                grip_angle = mc.get_pro_gripper_angle(GRIPPER_ID)
                if grip_angle is not None and 0 <= grip_angle <= 100:
                    current_gripper_angle = grip_angle
                    rospy.loginfo(f"[Pro450] 当前夹爪角度: {current_gripper_angle}°")
            except:
                pass
        else:
            rospy.logwarn("[Pro450] ⚠️  夹爪未检测到或初始化失败")
        
        return True
        
    except Exception as e:
        rospy.logerr(f"[Pro450] ❌ Pro450初始化失败: {e}")
        rospy.logerr("[Pro450] 请检查:")
        rospy.logerr("[Pro450] 1. 机械臂是否正确连接到网络")
        rospy.logerr(f"[Pro450] 2. IP地址 {PRO450_IP} 是否正确")
        rospy.logerr("[Pro450] 3. 机械臂是否已开机并正常工作")
        return False

# ========================= Gazebo发布 =========================
def publish_arm_to_gazebo(angles):
    """发布机械臂轨迹到Gazebo"""
    global pub_arm_command
    
    try:
        arm_traj = JointTrajectory()
        arm_traj.header.stamp = rospy.Time.now()
        arm_traj.joint_names = ARM_JOINTS
        
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(a) for a in angles]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        pt.accelerations = [0.0] * len(ARM_JOINTS)
        pt.time_from_start = rospy.Duration(0.5)
        arm_traj.points.append(pt)
        
        pub_arm_command.publish(arm_traj)
        rospy.logdebug(f"📤 Gazebo机械臂: {[round(a, 1) for a in angles]}°")
        
    except Exception as e:
        rospy.logwarn(f"Gazebo机械臂发布失败: {e}")

def publish_gripper_to_gazebo(gripper_angle):
    """发布夹爪轨迹到Gazebo"""
    global pub_gripper_command
    
    try:
        # 夹爪角度映射：0-100° -> -60°到60°
        mapped_gripper = ((gripper_angle - GRIPPER_MIN_ANGLE) /
                         (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE)) * \
                        (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
        
        # 限制范围
        mapped_gripper = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped_gripper))
        
        gripper_traj = JointTrajectory()
        gripper_traj.header.stamp = rospy.Time.now()
        gripper_traj.joint_names = [GRIPPER_JOINT]
        
        gp = JointTrajectoryPoint()
        gp.positions = [math.radians(mapped_gripper)]
        gp.velocities = [0.0]
        gp.accelerations = [0.0]
        gp.time_from_start = rospy.Duration(0.5)
        gripper_traj.points.append(gp)
        
        pub_gripper_command.publish(gripper_traj)
        rospy.logdebug(f"📤 Gazebo夹爪: {gripper_angle}° -> {mapped_gripper:.1f}°")
        
    except Exception as e:
        rospy.logwarn(f"Gazebo夹爪发布失败: {e}")

# ========================= Pro450 控制 =========================
def send_angles_to_pro450(angles):
    """发送角度到真实Pro450机械臂"""
    global mc
    
    if mc is None:
        return
    
    try:
        mc.send_angles(angles, ROBOT_SPEED)
        rospy.logdebug(f"📤 Pro450机械臂: {[round(a, 1) for a in angles]}°")
    except Exception as e:
        rospy.logwarn(f"Pro450机械臂发送失败: {e}")

def send_gripper_to_pro450(gripper_angle):
    """发送夹爪角度到真实Pro450"""
    global mc
    
    if mc is None:
        return
    
    try:
        # 限制范围 0-100
        gripper_angle = max(0, min(100, int(gripper_angle)))
        mc.set_pro_gripper_angle(gripper_angle, GRIPPER_ID)
        rospy.logdebug(f"📤 Pro450夹爪: {gripper_angle}°")
    except Exception as e:
        rospy.logwarn(f"Pro450夹爪发送失败: {e}")

# ========================= 命令队列处理 =========================
# 使用锁保护最新命令
command_lock = threading.Lock()
latest_arm_command = None
latest_gripper_command = None
command_event = threading.Event()

def command_executor_thread():
    """命令执行线程 - 只执行最新命令"""
    global executor_running, latest_arm_command, latest_gripper_command
    
    while executor_running and not rospy.is_shutdown():
        # 等待新命令
        if not command_event.wait(timeout=0.05):
            continue
        
        # 获取并清除最新命令
        with command_lock:
            arm_cmd = latest_arm_command
            gripper_cmd = latest_gripper_command
            latest_arm_command = None
            latest_gripper_command = None
            command_event.clear()
        
        # 执行最新的机械臂命令
        if arm_cmd is not None:
            publish_arm_to_gazebo(arm_cmd)
            if mc is not None:
                try:
                    mc.send_angles(arm_cmd, ROBOT_SPEED)
                except:
                    pass
        
        # 执行最新的夹爪命令
        if gripper_cmd is not None:
            publish_gripper_to_gazebo(gripper_cmd)
            if mc is not None:
                try:
                    gripper_angle = max(0, min(100, int(gripper_cmd)))
                    mc.set_pro_gripper_angle(gripper_angle, GRIPPER_ID)
                except:
                    pass

def add_arm_command(angles):
    """添加机械臂命令 - 直接覆盖旧命令"""
    global latest_arm_command
    with command_lock:
        latest_arm_command = angles[:]
        command_event.set()

def add_gripper_command(gripper_angle):
    """添加夹爪命令 - 直接覆盖旧命令"""
    global latest_gripper_command
    with command_lock:
        latest_gripper_command = gripper_angle
        command_event.set()

# ========================= 同步控制 =========================
def sync_arm(angles):
    """同步控制机械臂（Gazebo + Pro450）- 使用命令队列"""
    add_arm_command(angles)

def sync_gripper(gripper_angle):
    """同步控制夹爪（Gazebo + Pro450）- 使用命令队列"""
    add_gripper_command(gripper_angle)


# ========================= 键盘输入 =========================
class RawTerminal:
    """原始终端模式上下文管理器"""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.prev = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.prev)

def get_key_non_blocking():
    """非阻塞获取按键"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

# ========================= 键盘控制逻辑 =========================
def print_help():
    """打印帮助信息"""
    help_text = f"""
╔══════════════════════════════════════════════════════════╗
║   MyCobot Pro 450 键盘控制器 (Gazebo + 真实机械臂同步)   ║
╚══════════════════════════════════════════════════════════╝

关节控制 (普通步长: {ANGLE_STEP}°, 快速步长: {FAST_STEP}°):
  ┌─────────────────────────────────────────────────┐
  │ w/s: joint1 +/-     W/S: joint1 +/-  (快速)    │
  │ e/d: joint2 +/-     E/D: joint2 +/-  (快速)    │
  │ r/f: joint3 +/-     R/F: joint3 +/-  (快速)    │
  │ t/g: joint4 +/-     T/G: joint4 +/-  (快速)    │
  │ y/h: joint5 +/-     Y/H: joint5 +/-  (快速)    │
  │ u/j: joint6 +/-     U/J: joint6 +/-  (快速)    │
  └─────────────────────────────────────────────────┘

夹爪控制 (Pro力控夹爪 ID={GRIPPER_ID}):
  ┌─────────────────────────────────────────────────┐
  │ o: 夹爪完全打开 (100°)                          │
  │ p: 夹爪完全关闭 (0°)                            │
  │ [: 夹爪开启 +10°                                │
  │ ]: 夹爪关闭 -10°                                │
  └─────────────────────────────────────────────────┘

特殊命令:
  ┌─────────────────────────────────────────────────┐
  │ 1: 回到初始位置 (所有关节0°)                    │
  │ 2: 显示当前角度                                 │
  │ 3: 从真实机械臂读取当前角度                     │
  └─────────────────────────────────────────────────┘

❌ 退出:
  q: 退出程序

⚠️  安全提示:
  • 角度限制: ±180°
  • 程序会自动限制超出范围的角度
  • 同时控制 Gazebo 仿真和真实 Pro450 机械臂
"""
    print(help_text)

def read_current_from_robot():
    """从真实机械臂读取当前角度"""
    global mc, current_angles, current_gripper_angle
    
    if mc is None:
        print("❌ 真实机械臂未连接")
        return
    
    try:
        # 读取关节角度
        angles = mc.get_angles()
        if angles and len(angles) == 6:
            current_angles = angles[:]
            print(f"📍 从Pro450读取关节角度: {[round(a,1) for a in current_angles]}°")
        
        # 读取夹爪角度
        try:
            grip_angle = mc.get_pro_gripper_angle(GRIPPER_ID)
            if grip_angle is not None and 0 <= grip_angle <= 100:
                current_gripper_angle = grip_angle
                print(f"📍 从Pro450读取夹爪角度: {current_gripper_angle}°")
        except:
            pass
        
        # 同步到Gazebo
        publish_arm_to_gazebo(current_angles)
        publish_gripper_to_gazebo(current_gripper_angle)
        print("✅ 已同步到Gazebo")
        
    except Exception as e:
        print(f"❌ 读取失败: {e}")

def teleop_keyboard():
    """键盘控制主循环"""
    global current_angles, current_gripper_angle
    
    print_help()
    
    print(f"\n当前状态:")
    print(f"  关节角度: {[round(a, 1) for a in current_angles]}°")
    print(f"  夹爪角度: {current_gripper_angle}°\n")
    
    with RawTerminal():
        while not rospy.is_shutdown():
            key = get_key_non_blocking()
            
            if key is None:
                time.sleep(0.01)  # 短暂休眠减少CPU占用
                continue
            
            # 退出程序
            if key == 'q':
                print("\n退出程序...")
                break
            
            # 回到初始位置
            if key == '1':
                current_angles = home_pose.copy()
                sync_arm(current_angles)
                print(f"回到初始位置: {[round(a, 1) for a in current_angles]}°")
                continue
            
            # 显示当前角度
            if key == '2':
                print(f"📍 当前角度:")
                for i, angle in enumerate(current_angles):
                    print(f"   关节{i+1}: {angle:7.1f}°")
                print(f"   夹爪:   {current_gripper_angle:7.1f}°")
                continue
            
            # 从真实机械臂读取当前角度
            if key == '3':
                read_current_from_robot()
                continue
            
            # 夹爪控制
            if key == 'o':
                current_gripper_angle = 100
                sync_gripper(current_gripper_angle)
                print(f"夹爪打开: {current_gripper_angle}°")
                continue
            elif key == 'p':
                current_gripper_angle = 0
                sync_gripper(current_gripper_angle)
                print(f"夹爪关闭: {current_gripper_angle}°")
                continue
            elif key == '[':
                current_gripper_angle = min(100, current_gripper_angle + 10)
                sync_gripper(current_gripper_angle)
                print(f"夹爪开启: {current_gripper_angle}°")
                continue
            elif key == ']':
                current_gripper_angle = max(0, current_gripper_angle - 10)
                sync_gripper(current_gripper_angle)
                print(f"夹爪关闭: {current_gripper_angle}°")
                continue
            
            # 关节运动映射 (普通步长)
            normal_mapping = {
                'w': (0, +ANGLE_STEP), 's': (0, -ANGLE_STEP),
                'e': (1, +ANGLE_STEP), 'd': (1, -ANGLE_STEP),
                'r': (2, +ANGLE_STEP), 'f': (2, -ANGLE_STEP),
                't': (3, +ANGLE_STEP), 'g': (3, -ANGLE_STEP),
                'y': (4, +ANGLE_STEP), 'h': (4, -ANGLE_STEP),
                'u': (5, +ANGLE_STEP), 'j': (5, -ANGLE_STEP),
            }
            
            # 关节运动映射 (快速步长)
            fast_mapping = {
                'W': (0, +FAST_STEP), 'S': (0, -FAST_STEP),
                'E': (1, +FAST_STEP), 'D': (1, -FAST_STEP),
                'R': (2, +FAST_STEP), 'F': (2, -FAST_STEP),
                'T': (3, +FAST_STEP), 'G': (3, -FAST_STEP),
                'Y': (4, +FAST_STEP), 'H': (4, -FAST_STEP),
                'U': (5, +FAST_STEP), 'J': (5, -FAST_STEP),
            }
            
            # 检查普通步长映射
            if key in normal_mapping:
                idx, step = normal_mapping[key]
                current_angles[idx] += step
                current_angles = clamp_angles(current_angles)
                sync_arm(current_angles)
                print(f"🔧 关节{idx+1}: {current_angles[idx]:7.1f}° (步长: {step:+.1f}°)")
                continue
            
            # 检查快速步长映射
            if key in fast_mapping:
                idx, step = fast_mapping[key]
                current_angles[idx] += step
                current_angles = clamp_angles(current_angles)
                sync_arm(current_angles)
                print(f"🚀 关节{idx+1}: {current_angles[idx]:7.1f}° (快速: {step:+.1f}°)")
                continue

# ========================= 主函数 =========================
def main():
    global pub_arm_command, pub_gripper_command, mc, executor_running
    
    rospy.init_node("pro450_keyboard_control", anonymous=True)
    
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   MyCobot Pro 450 键盘控制器 (Gazebo + 真实机械臂同步)   ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    print("🚀 正在初始化...")
    
    # 初始化ROS发布者
    pub_arm_command = rospy.Publisher("/arm_controller/command", 
                                      JointTrajectory, queue_size=1)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command",
                                          JointTrajectory, queue_size=1)
    time.sleep(0.5)
    print("✅ ROS发布器初始化完成")
    
    # 初始化Pro450
    pro450_ok = initialize_pro450()
    if pro450_ok:
        print("✅ Pro450真实机械臂已连接")
    else:
        print("Pro450连接失败，仅控制Gazebo仿真")
    
    print(f"机器型号: MyCobot Pro 450")
    print(f"关节数量: {len(ARM_JOINTS)}")
    print(f"夹爪ID: {GRIPPER_ID}")
    print(f"⚡ 优化: 只执行最新命令，提高响应速度")
    print()
    
    # 启动命令执行线程
    executor_thread = threading.Thread(target=command_executor_thread, daemon=True)
    executor_thread.start()
    print("✅ 命令执行线程已启动")
    
    # 同步初始位置
    sync_arm(current_angles)
    sync_gripper(current_gripper_angle)
    
    print("✨ 准备就绪! 按 'q' 退出\n")
    
    # 注册退出处理
    def cleanup():
        global executor_running
        executor_running = False
        if mc is not None:
            try:
                rospy.loginfo("[main] 释放舵机...")
                mc.release_all_servos()
            except:
                pass
    
    rospy.on_shutdown(cleanup)
    
    try:
        teleop_keyboard()
    except KeyboardInterrupt:
        print("\n🛑 接收到中断信号，正在关闭...")
    except Exception as e:
        rospy.logerr(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()
        print("程序已退出")

if __name__ == "__main__":
    main()

