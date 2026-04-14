#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slider_control_450.py 
双模式滑块控制脚本 - MyCobot Pro 450版本：
  1: 滑块 -> Gazebo 控制器  
  2: 滑块 -> 真实 MyCobot Pro 450 机械臂
使用异步执行和频率控制优化性能，减少卡顿
集成MoveIt碰撞检测，确保运动安全
"""
import time
import math
import threading
import queue
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot import Pro450Client

# MoveIt碰撞检测相关导入
try:
    import moveit_commander
    from moveit_msgs.msg import RobotState, PlanningScene
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    rospy.logwarn("[slider_control] MoveIt未安装，将使用基础碰撞检测")

# 全局变量
mc = None
mode = 2
pub_arm = None
pub_gripper = None

# 末端坐标缓存（从coords_broadcaster订阅）
current_end_effector_coords = None
coords_lock = threading.Lock()

# 停止标志
is_stopped = False
stop_lock = threading.Lock()

# MoveIt碰撞检测相关全局变量
robot_commander = None
move_group = None
planning_scene_interface = None
state_validity_service = None
MOVEIT_COLLISION_CHECK = False  # 是否启用MoveIt碰撞检测

# 优化参数
ANGLE_THRESHOLD = 3.0           # 角度变化阈值(度)
GRIPPER_THRESHOLD = 5.0         # 夹爪角度变化阈值(度)  
MAX_COMMAND_RATE = 10.0         # 最大命令频率(Hz)
COMMAND_QUEUE_SIZE = 5          # 命令队列大小

# Pro450 连接参数
PRO450_IP = "192.168.0.232"
PRO450_PORT = 4500

# 安全角度限制 (度) - 根据更新后的Pro450 URDF定义 (2024.12)
JOINT_LIMITS = [
    (-162, 162),  # joint1 - 底座旋转 (±2.8274 rad)
    (-125, 125),  # joint2 - 大臂俯仰 (±2.1816 rad)
    (-154, 154),  # joint3 - 小臂俯仰 (±2.6878 rad)
    (-162, 162),  # joint4 - 腕部俯仰 (±2.8274 rad)
    (-162, 162),  # joint5 - 腕部旋转 (±2.8274 rad)
    (-165, 165),  # joint6 - 末端旋转 (±2.8797 rad)
]

GRIPPER_LIMITS = (0, 57.3)  # 夹爪角度限制

# 夹爪配置 - Pro力控夹爪
GRIPPER_ID = 14                 # Pro力控夹爪ID
GAZEBO_MIN_POSITION = 0 
GAZEBO_MAX_POSITION = 57.3
# Pro450夹爪角度范围
PRO450_GRIPPER_MIN = 0      # 夹爪关闭
PRO450_GRIPPER_MAX = 100    # 夹爪打开   

# 状态记录
last_angles = None
last_gripper_angle = None
last_command_time = 0
command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
is_executing = False

# 超限警告控制
last_warning_time = {}  # 每个关节的最后警告时间
WARNING_INTERVAL = 3.0  # 警告间隔(秒)

# 统计信息
stats = {
    'total_messages': 0,
    'commands_sent': 0,
    'commands_skipped': 0,
    'limit_violations': 0,
    'errors': 0
}

# 期望的关节名称
ARM_JOINTS = [
    "joint1",
    "joint2", 
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]
GRIPPER_JOINT = "gripper_controller"

def check_angle_limits(angles, gripper_angle):
    """检查角度是否在安全范围内"""
    global last_warning_time, stats
    
    current_time = time.time()
    violations = []
    
    # 检查关节角度限制
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        if angle < min_limit or angle > max_limit:
            joint_key = f"joint{i+1}"
            
            # 控制警告频率
            if joint_key not in last_warning_time or current_time - last_warning_time[joint_key] > WARNING_INTERVAL:
                violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°~{max_limit}°)")
                last_warning_time[joint_key] = current_time
                stats['limit_violations'] += 1
    
    # 检查夹爪角度限制
    min_grip, max_grip = GRIPPER_LIMITS
    if gripper_angle < min_grip or gripper_angle > max_grip:
        gripper_key = "gripper"
        
        if gripper_key not in last_warning_time or current_time - last_warning_time[gripper_key] > WARNING_INTERVAL:
            violations.append(f"夹爪: {gripper_angle:.1f}° (限制: {min_grip}°~{max_grip}°)")
            last_warning_time[gripper_key] = current_time
            stats['limit_violations'] += 1
    
    # 打印警告信息
    if violations:
        rospy.logwarn(f"[slider_control] ⚠️  角度超限:")
        for violation in violations:
            rospy.logwarn(f"[slider_control]    {violation}")
    
    return len(violations) == 0

# ========================= 碰撞检测 =========================
# 碰撞检测开关
COLLISION_CHECK_ENABLED = True

# 碰撞警告控制
last_collision_warning_time = 0
COLLISION_WARNING_INTERVAL = 2.0  # 碰撞警告间隔(秒)

# MoveIt碰撞检测配置
MOVEIT_GROUP_NAME = "arm"  # MoveIt规划组名称
GROUND_COLLISION_HEIGHT = 0.0  # 地面高度(m)，低于此高度视为碰撞

# 夹爪末端link名称（用于地面碰撞检测）
GRIPPER_TIP_LINKS = [
    "gripper_left1", "gripper_left2", "gripper_left3",
    "gripper_right1", "gripper_right2", "gripper_right3",
    "gripper_base", "gripper_connection"
]

# Pro450 实际关节限制（根据更新后的URDF 2024.12）
PRO450_JOINT_LIMITS = [
    (-162, 162),  # joint1 (±2.8274 rad)
    (-125, 125),  # joint2 (±2.1816 rad)
    (-154, 154),  # joint3 (±2.6878 rad)
    (-162, 162),  # joint4 (±2.8274 rad)
    (-162, 162),  # joint5 (±2.8274 rad)
    (-165, 165),  # joint6 (±2.8797 rad)
]

# ============ 夹爪和安全参数 ============
# 基于实际测量的MyCobot Pro 450连杆长度：
# - 底座到J1关节：21cm
# - J1到J2关节：18cm  
# - J2到J3关节：17.3cm
# - J3到J4关节：9.7cm
# - J4到J5（末端关节）：12cm
# - 夹爪长度（从J5向下）：17cm
# - J4到夹爪末端总长度：12+17 = 29cm
# 总计最大伸展长度：21+18+17.3+9.7+29 = 95cm

J4_TO_J5_LENGTH = 0.12          # J4到J5末端关节：12cm (实测)
GRIPPER_LENGTH = 0.17           # 夹爪长度：17cm (实测)
J4_TO_END_TOTAL = J4_TO_J5_LENGTH + GRIPPER_LENGTH  # 总计：29cm

MIN_END_HEIGHT = 0.30           # 末端最小安全高度 30cm (考虑29cm末端+安全余量)
MIN_END_HEIGHT_WARNING = 0.35   # 末端高度警告阈值 35cm
BASE_RADIUS = 0.12              # 底座半径约12cm，用于检测底座碰撞

# 地面高度（相对于base）
GROUND_LEVEL = -0.155  # base高度，地面在base下方

def estimate_end_effector_height(j2, j3, j4):
    """
    基于实际测量数据计算末端执行器高度（包含夹爪）
    实测数据：
    - 底座到J1=21cm, J1-J2=18cm, J2-J3=17.3cm, J3-J4=9.7cm
    - J4-J5=12cm, 夹爪长度=17cm
    - J4到夹爪末端总长度=29cm
    """
    j2_rad = math.radians(j2)
    j3_rad = math.radians(j3)
    j4_rad = math.radians(j4)
    
    # 基于实际测量的连杆长度 (单位: 米)
    BASE_TO_J1 = 0.21           # 底座到J1关节：21cm (实测)
    J1_TO_J2 = 0.18             # J1到J2关节：18cm (实测)
    J2_TO_J3 = 0.173            # J2到J3关节：17.3cm (实测)
    J3_TO_J4 = 0.097            # J3到J4关节：9.7cm (实测)
    J4_TO_END = 0.29            # J4到夹爪末端：29cm (12cm+17cm) (实测) ✅
    
    # 累积角度计算
    angle2 = j2_rad
    angle3 = angle2 + j3_rad
    angle4 = angle3 + j4_rad
    
    # 高度计算 (正向运动学) - 基于完整实测数据
    height = BASE_TO_J1         # 底座到J1
    height += J1_TO_J2          # J1到J2 (垂直)
    height += J2_TO_J3 * math.cos(angle2)  # J2到J3的垂直分量
    height += J3_TO_J4 * math.cos(angle3)  # J3到J4的垂直分量
    height += J4_TO_END * math.cos(angle4)  # J4到夹爪末端的垂直分量 (包含夹爪)
    
    return height

def estimate_end_effector_distance_to_base(j2, j3, j4):
    """
    基于实际测量数据估算末端执行器到底座中心的水平距离
    用于检测夹爪是否可能撞到底座
    """
    j2_rad = math.radians(j2)
    j3_rad = math.radians(j3)
    j4_rad = math.radians(j4)
    
    # 基于实际测量的连杆长度
    J2_TO_J3 = 0.173            # J2到J3：17.3cm (实测)
    J3_TO_J4 = 0.097            # J3到J4：9.7cm (实测)
    J4_TO_END = 0.29            # J4到夹爪末端：29cm (12cm+17cm) (实测) ✅
    
    angle2 = j2_rad
    angle3 = angle2 + j3_rad
    angle4 = angle3 + j4_rad
    
    # 计算水平距离（XY平面）
    distance = abs(J2_TO_J3 * math.sin(angle2))
    distance += abs(J3_TO_J4 * math.sin(angle3))
    distance += abs(J4_TO_END * math.sin(angle4))
    
    return distance

def validate_height_calculation(j2, j3, j4):
    """
    验证实测数据与URDF数据的高度计算差异
    用于调试和验证
    """
    # 新的实测数据计算
    new_height = estimate_end_effector_height(j2, j3, j4)
    
    # URDF数据计算
    j2_rad = math.radians(j2)
    j3_rad = math.radians(j3)
    j4_rad = math.radians(j4)
    
    angle2 = j2_rad
    angle3 = angle2 + j3_rad
    angle4 = angle3 + j4_rad
    
    urdf_height = 0.155 + 0.048  # URDF: base + L1
    urdf_height += 0.18 * math.cos(angle2)   # URDF: L2
    urdf_height += 0.1735 * math.cos(angle3) # URDF: L3
    urdf_height += 0.08 * math.cos(angle4)   # URDF: L4 (简化)
    urdf_height += 0.17 * math.cos(angle4)   # URDF: L5 (保守估计)
    
    diff = new_height - urdf_height
    
    rospy.logdebug(f"[高度验证] j2={j2:.0f}°, j3={j3:.0f}°, j4={j4:.0f}°")
    rospy.logdebug(f"[高度验证] 实测算法: {new_height*1000:.0f}mm, URDF算法: {urdf_height*1000:.0f}mm, 差异: {diff*1000:.0f}mm")
    
    return new_height, urdf_height, diff

# ========================= MoveIt碰撞检测 =========================
def initialize_moveit_collision_checker():
    """初始化MoveIt碰撞检测"""
    global robot_commander, move_group, planning_scene_interface, state_validity_service, MOVEIT_COLLISION_CHECK
    
    if not MOVEIT_AVAILABLE:
        rospy.logwarn("[碰撞检测] MoveIt不可用，使用基础碰撞检测")
        return False
    
    try:
        rospy.loginfo("[碰撞检测] 正在初始化MoveIt碰撞检测...")
        
        # 初始化moveit_commander
        moveit_commander.roscpp_initialize([])
        
        # 创建RobotCommander对象
        robot_commander = moveit_commander.RobotCommander()
        
        # 创建MoveGroupCommander对象
        move_group = moveit_commander.MoveGroupCommander(MOVEIT_GROUP_NAME)
        
        # 创建PlanningSceneInterface对象
        planning_scene_interface = moveit_commander.PlanningSceneInterface()
        
        # 等待状态有效性服务
        rospy.loginfo("[碰撞检测] 等待 /check_state_validity 服务...")
        try:
            rospy.wait_for_service('/check_state_validity', timeout=5.0)
            state_validity_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
            rospy.loginfo("[碰撞检测] ✅ 状态有效性服务已连接")
        except rospy.ROSException:
            rospy.logwarn("[碰撞检测] /check_state_validity 服务不可用，将使用基础检测")
            state_validity_service = None
        
        # 添加地面碰撞对象
        add_ground_collision_object()
        
        MOVEIT_COLLISION_CHECK = True
        rospy.loginfo("[碰撞检测] ✅ MoveIt碰撞检测初始化成功")
        rospy.loginfo(f"[碰撞检测]    规划组: {MOVEIT_GROUP_NAME}")
        rospy.loginfo(f"[碰撞检测]    末端执行器: {move_group.get_end_effector_link()}")
        
        return True
        
    except Exception as e:
        rospy.logwarn(f"[碰撞检测] MoveIt初始化失败: {e}")
        rospy.logwarn("[碰撞检测] 将使用基础碰撞检测")
        MOVEIT_COLLISION_CHECK = False
        return False

def add_ground_collision_object():
    """添加地面碰撞对象到规划场景"""
    global planning_scene_interface
    
    if planning_scene_interface is None:
        return
    
    try:
        from geometry_msgs.msg import PoseStamped
        
        # 创建地面平面
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "world"
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = -0.01  # 地面略低于0
        ground_pose.pose.orientation.w = 1.0
        
        # 添加一个大的盒子作为地面
        planning_scene_interface.add_box(
            "ground_plane",
            ground_pose,
            size=(3.0, 3.0, 0.02)  # 3m x 3m x 2cm 的地面
        )
        
        rospy.loginfo("[碰撞检测] ✅ 已添加地面碰撞对象")
        
    except Exception as e:
        rospy.logwarn(f"[碰撞检测] 添加地面碰撞对象失败: {e}")

def check_moveit_collision(angles):
    """
    使用MoveIt检查关节角度是否会导致碰撞
    
    参数:
        angles: 6个关节角度(度)
    
    返回:
        (is_safe, collision_info): 是否安全，碰撞信息
    """
    global robot_commander, move_group, state_validity_service
    
    if not MOVEIT_COLLISION_CHECK or move_group is None:
        return True, ""
    
    try:
        # 将角度转换为弧度
        joint_positions = [math.radians(a) for a in angles]
        
        # 方法1: 使用状态有效性服务（更准确）
        if state_validity_service is not None:
            return check_state_validity_service(joint_positions)
        
        # 方法2: 使用MoveGroup的碰撞检测
        return check_moveit_group_collision(joint_positions)
        
    except Exception as e:
        rospy.logdebug(f"[碰撞检测] MoveIt检测异常: {e}")
        return True, ""  # 异常时允许通过，由基础检测兜底

def check_state_validity_service(joint_positions):
    """使用/check_state_validity服务检查碰撞"""
    global robot_commander, state_validity_service
    
    try:
        # 创建RobotState消息
        robot_state = RobotState()
        robot_state.joint_state.name = ARM_JOINTS
        robot_state.joint_state.position = joint_positions
        
        # 创建请求
        request = GetStateValidityRequest()
        request.robot_state = robot_state
        request.group_name = MOVEIT_GROUP_NAME
        
        # 调用服务
        response = state_validity_service(request)
        
        if not response.valid:
            # 解析碰撞信息
            collision_pairs = []
            for contact in response.contacts:
                pair = f"{contact.contact_body_1} <-> {contact.contact_body_2}"
                if pair not in collision_pairs:
                    collision_pairs.append(pair)
            
            collision_info = ", ".join(collision_pairs[:3])  # 最多显示3对
            return False, f"碰撞: {collision_info}"
        
        return True, ""
        
    except Exception as e:
        rospy.logdebug(f"[碰撞检测] 服务调用失败: {e}")
        return True, ""

def check_moveit_group_collision(joint_positions):
    """使用MoveGroup检查碰撞"""
    global move_group
    
    try:
        # 获取当前状态
        current_state = move_group.get_current_state()
        
        # 设置关节值
        joint_state = current_state.joint_state
        for i, joint_name in enumerate(ARM_JOINTS):
            if joint_name in joint_state.name:
                idx = joint_state.name.index(joint_name)
                joint_state.position = list(joint_state.position)
                joint_state.position[idx] = joint_positions[i]
        
        # 检查是否在关节限制内
        # MoveIt会自动检查碰撞
        move_group.set_joint_value_target(joint_positions)
        
        # 尝试规划（不执行）来检测碰撞
        plan = move_group.plan()
        
        # 清除目标
        move_group.clear_pose_targets()
        
        # 检查规划是否成功
        if isinstance(plan, tuple):
            success = plan[0]
        else:
            success = plan is not None
        
        if not success:
            return False, "MoveIt规划失败(可能碰撞)"
        
        return True, ""
        
    except Exception as e:
        rospy.logdebug(f"[碰撞检测] MoveGroup检测失败: {e}")
        return True, ""

def check_gripper_ground_collision(angles):
    """
    检查夹爪是否会与地面碰撞
    
    使用正向运动学估算夹爪末端位置，检查是否低于地面
    """
    j1, j2, j3, j4, j5, j6 = angles
    
    # 估算末端高度
    end_height = estimate_end_effector_height(j2, j3, j4)
    
    # 检查是否低于地面安全高度
    if end_height < MIN_END_HEIGHT:
        return False, f"夹爪过低: {end_height*100:.0f}cm (最小: {MIN_END_HEIGHT*100:.0f}cm)"
    
    return True, ""

def check_collision(angles):
    """
    检查关节角度组合是否可能导致碰撞
    返回: (is_safe, warning_message)
    
    碰撞检测策略:
    1. MoveIt碰撞检测（如果可用）- 检测自碰撞和环境碰撞
    2. 夹爪地面碰撞检测 - 确保夹爪不会撞地
    3. 基础规则检测 - 作为兜底保护
    
    Pro450 碰撞风险区域（针对17cm夹爪优化）：
    1. 末端撞地：当末端高度低于安全高度时
    2. 自碰撞：当机械臂折叠回自身时
    3. 底座碰撞：当夹爪靠近底座时
    4. 关节极限：超出URDF定义的关节限制
    """
    global last_collision_warning_time
    
    if not COLLISION_CHECK_ENABLED:
        return True, ""
    
    j1, j2, j3, j4, j5, j6 = angles
    warnings = []
    
    # ===== MoveIt碰撞检测（优先级最高）=====
    if MOVEIT_COLLISION_CHECK:
        moveit_safe, moveit_info = check_moveit_collision(angles)
        if not moveit_safe:
            warnings.append(f"🚨 MoveIt检测: {moveit_info}")
    
    # ===== 夹爪地面碰撞检测 =====
    gripper_safe, gripper_info = check_gripper_ground_collision(angles)
    if not gripper_safe:
        warnings.append(f"🚨 {gripper_info}")
    
    # ===== 规则0: 检查URDF关节限制 =====
    for i, (angle, (min_lim, max_lim)) in enumerate(zip(angles, PRO450_JOINT_LIMITS)):
        if angle < min_lim or angle > max_lim:
            warnings.append(f"⚠️ 关节{i+1}超出URDF限制: {angle:.0f}° (限制: {min_lim}°~{max_lim}°)")
    
    # ===== 规则1: 末端撞地检测（核心规则，针对17cm夹爪）=====
    end_height = estimate_end_effector_height(j2, j3, j4)
    if end_height < MIN_END_HEIGHT:
        warnings.append(f"🚨 末端过低会撞地: 高度≈{end_height*100:.0f}cm (最小安全高度: {MIN_END_HEIGHT*100:.0f}cm)")
    elif end_height < MIN_END_HEIGHT_WARNING:
        warnings.append(f"⚠️ 末端接近地面: 高度≈{end_height*100:.0f}cm")
    
    # ===== 规则2: 防止向前过度伸展撞地 =====
    # 当j2向前倾斜且j3也向前时，17cm夹爪更容易撞地
    # 收紧阈值：j2>45° 且 j3>20° 时警告
    if j2 > 45 and j3 > 20:
        warnings.append(f"⚠️ 向前过度伸展(夹爪可能撞地): j2={j2:.0f}°, j3={j3:.0f}°")
    
    # ===== 规则3: 防止向下折叠撞地 =====
    # 当j2向前且j3向下弯曲时，收紧阈值
    if j2 > 20 and j3 < -70:
        warnings.append(f"⚠️ 向下折叠可能撞地: j2={j2:.0f}°, j3={j3:.0f}°")
    
    # ===== 规则4: 防止自碰撞 - 机械臂折叠回自身 =====
    # 当j2向后且j3向上折叠时，17cm夹爪更容易撞到大臂
    if j2 < -45 and j3 > 100:
        warnings.append(f"⚠️ 可能自碰撞(夹爪撞大臂): j2={j2:.0f}°, j3={j3:.0f}°")
    
    # ===== 规则5: 防止夹爪撞底座 =====
    # 当机械臂向下且向内折叠时，检测夹爪到底座距离
    if j2 > 60 and j3 < -45:
        base_distance = estimate_end_effector_distance_to_base(j2, j3, j4)
        if base_distance < BASE_RADIUS + 0.05:  # 底座半径 + 5cm安全余量
            warnings.append(f"🚨 夹爪可能撞底座: j2={j2:.0f}°, j3={j3:.0f}°")
    
    # ===== 规则6: 防止极端组合姿态 =====
    # 当j2和j3同时接近极限时，收紧阈值（从90/120改为80/100）
    if abs(j2) > 80 and abs(j3) > 100:
        warnings.append(f"🚨 极端姿态(危险): j2={j2:.0f}°, j3={j3:.0f}°")
    
    # ===== 规则7: 防止j4导致夹爪撞地或撞臂 =====
    # 当j4角度过大时，17cm夹爪可能撞到其他部位（从120改为100）
    if abs(j4) > 100:
        warnings.append(f"⚠️ 腕部角度过大(夹爪可能碰撞): j4={j4:.0f}°")
    
    # ===== 规则8: 组合危险姿态检测 =====
    # j2向前 + j3向下 + j4向下 = 夹爪直指地面
    if j2 > 30 and j3 < -30 and j4 < -60:
        warnings.append(f"🚨 夹爪指向地面: j2={j2:.0f}°, j3={j3:.0f}°, j4={j4:.0f}°")
    
    # ===== 规则9: 防止向后下方折叠撞地 =====
    # 当j2向后(负值)且j3也向下(负值)时，夹爪可能撞地
    # j2=-116°, j3=-124° 这种情况已经碰撞了
    if j2 < -60 and j3 < -80:
        warnings.append(f"🚨 向后下方折叠(夹爪撞地): j2={j2:.0f}°, j3={j3:.0f}°")
    
    if warnings:
        current_time = time.time()
        if current_time - last_collision_warning_time > COLLISION_WARNING_INTERVAL:
            last_collision_warning_time = current_time
            for w in warnings:
                rospy.logwarn(f"[碰撞检测] {w}")
        return False, warnings[0]
    
    return True, ""

def adjust_for_collision(angles):
    """
    调整角度以避免碰撞（针对17cm夹爪优化）
    返回调整后的安全角度
    """
    j1, j2, j3, j4, j5, j6 = angles[:]
    adjusted = False
    
    # ===== 首先应用URDF关节限制 =====
    for i, (min_lim, max_lim) in enumerate(PRO450_JOINT_LIMITS):
        if angles[i] < min_lim:
            angles[i] = min_lim
            adjusted = True
        elif angles[i] > max_lim:
            angles[i] = max_lim
            adjusted = True
    
    j1, j2, j3, j4, j5, j6 = angles
    
    # ===== 规则1: 限制末端高度（核心调整）=====
    end_height = estimate_end_effector_height(j2, j3, j4)
    if end_height < MIN_END_HEIGHT:
        # 策略1: 先尝试调整j3（小臂向上）
        if j3 < 0:
            j3 = min(j3 + 30, 0)  # 小臂向上抬
            adjusted = True
        # 策略2: 如果j3已经调整，再调整j2
        if estimate_end_effector_height(j2, j3, j4) < MIN_END_HEIGHT:
            j2 = max(j2 - 20, -60)  # 大臂向后
            adjusted = True
        # 策略3: 调整j4
        if estimate_end_effector_height(j2, j3, j4) < MIN_END_HEIGHT:
            if j4 < 0:
                j4 = min(j4 + 30, 60)
            adjusted = True
    
    # ===== 规则2: 限制向前过度伸展 =====
    if j2 > 45 and j3 > 20:
        j3 = min(j3, 20 - (j2 - 45) * 0.8)
        adjusted = True
    
    # ===== 规则3: 限制向下折叠 =====
    if j2 > 20 and j3 < -70:
        j3 = max(j3, -70)
        adjusted = True
    
    # ===== 规则4: 防止自碰撞 =====
    if j2 < -45 and j3 > 100:
        j3 = min(j3, 100)
        adjusted = True
    
    # ===== 规则5: 防止夹爪撞底座 =====
    if j2 > 60 and j3 < -45:
        j3 = max(j3, -45)
        adjusted = True
    
    # ===== 规则6: 限制j4角度（从120改为100）=====
    if abs(j4) > 100:
        j4 = max(-100, min(100, j4))
        adjusted = True
    
    # ===== 规则7: 防止夹爪指向地面 =====
    if j2 > 30 and j3 < -30 and j4 < -60:
        j4 = max(j4, -60)
        adjusted = True
    
    # ===== 规则8: 防止极端姿态（从90/120改为80/100）=====
    if abs(j2) > 80 and abs(j3) > 100:
        # 限制j3的绝对值不超过100
        if j3 > 100:
            j3 = 100
        elif j3 < -100:
            j3 = -100
        adjusted = True
    
    # ===== 规则9: 防止向后下方折叠撞地 =====
    # 当j2向后(负值)且j3也向下(负值)时，限制j3
    if j2 < -60 and j3 < -80:
        j3 = max(j3, -80)  # 限制j3不低于-80°
        adjusted = True
    
    if adjusted:
        rospy.loginfo(f"[碰撞检测] 已调整角度以避免碰撞 (17cm夹爪保护)")
        rospy.loginfo(f"[碰撞检测] 调整后: j2={j2:.0f}°, j3={j3:.0f}°, j4={j4:.0f}°")
    
    return [j1, j2, j3, j4, j5, j6]

def clamp_angles(angles, gripper_angle):
    """将角度限制在安全范围内"""
    # 限制关节角度
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angle = max(min_limit, min(max_limit, angle))
        clamped_angles.append(clamped_angle)
    
    # 碰撞检测和调整
    is_safe, warning = check_collision(clamped_angles)
    if not is_safe:
        clamped_angles = adjust_for_collision(clamped_angles)
    
    # 限制夹爪角度
    min_grip, max_grip = GRIPPER_LIMITS
    clamped_gripper = max(min_grip, min(max_grip, gripper_angle))
    
    return clamped_angles, clamped_gripper

def calculate_angle_difference(angles1, angles2):
    """计算角度差异"""
    if angles1 is None or angles2 is None:
        return float('inf')
    return sum(abs(a - b) for a, b in zip(angles1, angles2))

def should_send_command(new_angles, new_gripper_angle):
    """判断是否应该发送命令"""
    global last_angles, last_gripper_angle, last_command_time
    
    current_time = time.time()
    
    # 频率限制
    if current_time - last_command_time < 1.0 / MAX_COMMAND_RATE:
        return False, "频率限制"
    
    # 角度变化检查
    angle_diff = calculate_angle_difference(new_angles, last_angles)
    gripper_diff = abs(new_gripper_angle - last_gripper_angle) if last_gripper_angle is not None else float('inf')
    
    if angle_diff < ANGLE_THRESHOLD and gripper_diff < GRIPPER_THRESHOLD:
        return False, f"角度变化太小 (臂:{angle_diff:.1f}°, 夹爪:{gripper_diff:.1f}°)"
    
    return True, "允许发送"

class RobotCommand:
    """机器人命令类"""
    def __init__(self, cmd_type, data, timestamp=None):
        self.type = cmd_type  # 'angles' or 'gripper'
        self.data = data
        self.timestamp = timestamp or time.time()

def is_pro450_connected():
    """检查Pro450是否连接"""
    global mc
    try:
        if mc is None:
            return False
        mc.get_angles()
        return True
    except Exception as e:
        return False

def add_command_to_queue(command):
    """添加命令到队列"""
    try:
        command_queue.put_nowait(command)
        return True
    except queue.Full:
        # 队列满时移除最旧命令
        try:
            old_command = command_queue.get_nowait()
            command_queue.put_nowait(command)
            return True
        except queue.Empty:
            return False

def initialize_gripper():
    """初始化Pro力控夹爪"""
    global mc
    
    try:
        rospy.loginfo("[夹爪] 尝试初始化Pro力控夹爪...")
        time.sleep(2)
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

def map_gripper_angle_to_pro450(gazebo_angle):
    """将Gazebo夹爪角度映射到Pro450夹爪角度"""
    # Gazebo角度范围: -60 到 60
    # Pro450夹爪角度范围: 0 到 100
    # 映射: -60° -> 0 (关闭), 60° -> 100 (打开)
    
    # 限制输入范围
    gazebo_angle = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, gazebo_angle))
    
    # 线性映射
    mapped_angle = ((gazebo_angle - GAZEBO_MIN_POSITION) / 
                    (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION)) * \
                   (PRO450_GRIPPER_MAX - PRO450_GRIPPER_MIN) + PRO450_GRIPPER_MIN
    
    # 限制到0-100范围并转为整数
    mapped_angle = int(round(max(PRO450_GRIPPER_MIN, min(PRO450_GRIPPER_MAX, mapped_angle))))
    
    return mapped_angle

def set_gripper_angle_pro450(gazebo_angle):
    """设置Pro450夹爪角度 - 使用Pro力控夹爪接口"""
    global mc
    
    # 映射角度
    mapped_angle = map_gripper_angle_to_pro450(gazebo_angle)
    
    try:
        # 使用Pro力控夹爪角度设置
        mc.set_pro_gripper_angle(mapped_angle, GRIPPER_ID)
        rospy.logdebug(f"[夹爪] Pro力控夹爪设置成功: Gazebo={gazebo_angle:.1f}° → Pro450={mapped_angle}°")
        return True
    except TypeError as te:
        rospy.logwarn(f"[夹爪] set_pro_gripper_angle参数错误: {te}")
        # 备选方案：使用基础夹爪状态控制
        if mapped_angle >= 90:
            mc.set_gripper_state(1, 80)  # 状态1 = 打开
            rospy.loginfo(f"[夹爪] 状态控制: 打开 (角度: {mapped_angle}°)")
        elif mapped_angle <= 10:
            mc.set_gripper_state(0, 80)  # 状态0 = 关闭
            rospy.loginfo(f"[夹爪] 状态控制: 关闭 (角度: {mapped_angle}°)")
        else:
            rospy.logwarn(f"[夹爪] 中间角度 {mapped_angle}° 无法用状态控制")
        return True
    except Exception as e:
        rospy.logwarn(f"[夹爪] 设置夹爪失败: {e}")
        return False

def command_executor():
    """异步命令执行线程 - 优化版：只执行最新命令"""
    global is_executing, last_angles, last_gripper_angle, last_command_time, stats
    global current_end_effector_coords, coords_lock, is_stopped, stop_lock
    
    while not rospy.is_shutdown():
        try:
            # 等待队列有数据（阻塞等待，但有超时）
            try:
                first_cmd = command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            
            # 收集所有待处理命令，包括刚取出的第一个
            latest_angles_cmd = None
            latest_gripper_cmd = None
            
            # 处理第一个命令
            if first_cmd.type == 'angles':
                latest_angles_cmd = first_cmd
            elif first_cmd.type == 'gripper':
                latest_gripper_cmd = first_cmd
            
            # 获取队列中剩余的最新命令（丢弃旧命令）
            while True:
                try:
                    cmd = command_queue.get_nowait()
                    if cmd.type == 'angles':
                        latest_angles_cmd = cmd
                    elif cmd.type == 'gripper':
                        latest_gripper_cmd = cmd
                except queue.Empty:
                    break
            
            if not is_pro450_connected():
                rospy.logwarn("[slider_control] Pro450未连接，跳过命令")
                stats['errors'] += 1
                continue
            
            is_executing = True
            
            try:
                # 执行最新的角度命令
                if latest_angles_cmd is not None:
                    # 计算目标高度（基于命令的关节角度）
                    MIN_SAFE_HEIGHT = 300  # 最小安全高度 300mm (基于29cm末端+安全余量)
                    
                    try:
                        # 提取关节角度
                        j2 = latest_angles_cmd.data[1]  # joint2
                        j3 = latest_angles_cmd.data[2]  # joint3
                        j4 = latest_angles_cmd.data[3]  # joint4
                        
                        # 计算目标高度（单位：米）
                        target_height_m = estimate_end_effector_height(j2, j3, j4)
                        target_height = target_height_m * 1000  # 转换为mm
                        
                        # 验证新旧算法差异（调试用）
                        if rospy.get_param('/slider_control/debug_height', False):
                            validate_height_calculation(j2, j3, j4)
                        
                        # 获取当前高度
                        current_height = None
                        with coords_lock:
                            if current_end_effector_coords is not None:
                                current_height = current_end_effector_coords.z
                        
                        # 关键逻辑：检查目标高度，而不是当前高度
                        if target_height < MIN_SAFE_HEIGHT:
                            # 目标高度过低，拒绝命令
                            rospy.logwarn(f"[slider_control] 🚨 目标高度过低: {target_height:.0f}mm < {MIN_SAFE_HEIGHT}mm")
                            rospy.logwarn(f"[slider_control] ⚠️  此命令会导致夹爪撞地！拒绝执行")
                            stats['commands_skipped'] += 1
                            continue  # 跳过此命令
                        else:
                            # 目标高度安全，允许执行
                            if current_height is not None and current_height < MIN_SAFE_HEIGHT:
                                # 当前高度低，但目标高度安全 - 这是恢复运动
                                rospy.loginfo(f"[slider_control] 🔄 恢复运动: 当前{current_height:.0f}mm → 目标{target_height:.0f}mm")
                                rospy.loginfo(f"[slider_control] ✅ 目标高度安全，允许执行恢复命令")
                                
                                # 清除停止状态，允许恢复
                                with stop_lock:
                                    if is_stopped:
                                        is_stopped = False
                                        rospy.loginfo(f"[slider_control] 🔓 已解除停止状态，机械臂恢复运动")
                            else:
                                rospy.logdebug(f"[slider_control] 目标高度: {target_height:.0f}mm (安全)")
                    
                    except Exception as e:
                        rospy.logwarn(f"[slider_control] 高度计算异常: {e}，继续执行")
                    
                    rospy.loginfo(f"[slider_control]  发送角度到Pro450: {[round(a,1) for a in latest_angles_cmd.data]}")
                    mc.send_angles(latest_angles_cmd.data, 10)  # 提高速度到50
                    last_angles = latest_angles_cmd.data.copy()
                    stats['commands_sent'] += 1
                
                # 执行最新的夹爪命令
                if latest_gripper_cmd is not None:
                    gazebo_angle = latest_gripper_cmd.data
                    mapped_angle = map_gripper_angle_to_pro450(gazebo_angle)
                    
                    rospy.loginfo(f"[slider_control]  夹爪控制: Gazebo={gazebo_angle:.1f}° → Pro450={mapped_angle}°")
                    
                    if set_gripper_angle_pro450(gazebo_angle):
                        last_gripper_angle = gazebo_angle
                        stats['commands_sent'] += 1
                        rospy.loginfo(f"[slider_control] ✅ 夹爪控制成功: {mapped_angle}°")
                    else:
                        rospy.logwarn(f"[slider_control] ❌ 夹爪控制失败")
                        stats['errors'] += 1
                
                last_command_time = time.time()
                
            except Exception as e:
                rospy.logerr(f"[slider_control] 命令执行失败: {e}")
                stats['errors'] += 1
                
            finally:
                is_executing = False
                
        except Exception as e:
            rospy.logerr(f"[slider_control] 命令执行器错误: {e}")
            is_executing = False

def coords_callback(msg):
    """末端坐标回调函数"""
    global current_end_effector_coords, coords_lock
    
    with coords_lock:
        current_end_effector_coords = msg
    
    rospy.logdebug(f"[slider_control] 收到末端坐标: X={msg.x:.1f}mm, Y={msg.y:.1f}mm, Z={msg.z:.1f}mm")

def monitor_height():
    """实时监控末端高度，如果过低则停止机械臂，高于安全高度则自动恢复"""
    global current_end_effector_coords, coords_lock, mc, mode, pub_arm, is_stopped, stop_lock
    
    MIN_SAFE_HEIGHT = 300  # 最小安全高度 300mm (基于29cm末端+安全余量)
    RECOVERY_HEIGHT = 300  # 恢复高度阈值 300mm (高于此高度自动恢复)
    last_warning_time = 0
    last_recovery_log_time = 0
    WARNING_INTERVAL = 1.0  # 警告间隔(秒)
    RECOVERY_LOG_INTERVAL = 2.0  # 恢复日志间隔(秒)
    
    rate = rospy.Rate(20)  # 20Hz 监控频率
    
    while not rospy.is_shutdown():
        try:
            with coords_lock:
                if current_end_effector_coords is not None:
                    end_height = current_end_effector_coords.z
                    
                    # 检查是否需要停止
                    if end_height < MIN_SAFE_HEIGHT:
                        current_time = time.time()
                        if current_time - last_warning_time > WARNING_INTERVAL:
                            rospy.logwarn(f"[高度监控] 🚨 末端高度过低: {end_height}mm < {MIN_SAFE_HEIGHT}mm")
                            rospy.logwarn(f"[高度监控] ⚠️  立即停止机械臂！")
                            last_warning_time = current_time
                        
                        # 设置停止标志
                        with stop_lock:
                            is_stopped = True
                        
                        # 根据模式停止机械臂
                        if mode == 2:
                            # 真实机械臂模式
                            try:
                                mc.stop()
                                rospy.logwarn(f"[高度监控] ✋ 真实机械臂已停止")
                            except Exception as e:
                                rospy.logerr(f"[高度监控] 停止真实机械臂失败: {e}")
                        
                        elif mode == 1:
                            # Gazebo仿真模式 - 发布空轨迹停止
                            try:
                                stop_traj = JointTrajectory()
                                stop_traj.header.stamp = rospy.Time.now()
                                stop_traj.joint_names = ARM_JOINTS
                                
                                pt = JointTrajectoryPoint()
                                pt.positions = [0.0] * 6  # 所有关节停止
                                pt.velocities = [0.0] * 6
                                pt.accelerations = [0.0] * 6
                                pt.time_from_start = rospy.Duration(0.0)
                                stop_traj.points.append(pt)
                                
                                pub_arm.publish(stop_traj)
                                rospy.logwarn(f"[高度监控] ✋ Gazebo机械臂已停止")
                            except Exception as e:
                                rospy.logerr(f"[高度监控] 停止Gazebo机械臂失败: {e}")
                    
                    # 检查是否可以恢复运动
                    elif end_height >= RECOVERY_HEIGHT:
                        with stop_lock:
                            if is_stopped:
                                # 恢复运动
                                is_stopped = False
                                current_time = time.time()
                                if current_time - last_recovery_log_time > RECOVERY_LOG_INTERVAL:
                                    rospy.loginfo(f"[高度监控] ✅ 末端高度恢复安全: {end_height}mm >= {RECOVERY_HEIGHT}mm")
                                    rospy.loginfo(f"[高度监控] 🔄 机械臂已恢复，可以继续运动")
                                    last_recovery_log_time = current_time
        except Exception as e:
            rospy.logerr_throttle(5, f"[高度监控] 监控错误: {e}")
        
        rate.sleep()

def callback(msg: JointState):
    """优化的回调函数"""
    global stats
    
    stats['total_messages'] += 1
    
    # 快速解析关节数据
    arm_deg = [0.0] * len(ARM_JOINTS)
    grip_deg = 0.0
    
    name_to_deg = {name: math.degrees(pos) for name, pos in zip(msg.name, msg.position)}
    
    # 提取臂关节角度
    for i, joint_name in enumerate(ARM_JOINTS):
        if joint_name in name_to_deg:
            arm_deg[i] = round(name_to_deg[joint_name], 1)
    
    # 提取夹爪角度
    if GRIPPER_JOINT in name_to_deg:
        grip_deg = round(name_to_deg[GRIPPER_JOINT], 1)
    
    # 检查角度限制
    check_angle_limits(arm_deg, grip_deg)
    
    if mode == 1:
        # Gazebo模式
        should_send, reason = should_send_command(arm_deg, grip_deg)
        
        if should_send:
            clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
            publish_to_gazebo(clamped_arm, clamped_grip)
        else:
            stats['commands_skipped'] += 1
            rospy.logdebug(f"[slider_control] 跳过命令: {reason}")
        
    elif mode == 2:
        # 真实机械臂模式 - 异步处理
        should_send, reason = should_send_command(arm_deg, grip_deg)
        
        if should_send:
            # 对真实机械臂进行角度限制
            clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
            
            # 添加臂关节命令
            arm_command = RobotCommand('angles', clamped_arm)
            if add_command_to_queue(arm_command):
                # 添加夹爪命令（如果角度变化足够大）
                gripper_diff = abs(clamped_grip - last_gripper_angle) if last_gripper_angle is not None else float('inf')
                if gripper_diff >= GRIPPER_THRESHOLD:
                    gripper_command = RobotCommand('gripper', clamped_grip)
                    add_command_to_queue(gripper_command)
            else:
                stats['commands_skipped'] += 1
        else:
            stats['commands_skipped'] += 1
            rospy.logdebug(f"[slider_control] 跳过命令: {reason}")

def publish_to_gazebo(arm_deg, grip_deg):
    """发布到Gazebo"""
    global pub_arm, pub_gripper, last_angles, last_gripper_angle, last_command_time, stats
    
    try:
        # 臂关节轨迹
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(d) for d in arm_deg]
        pt.time_from_start = rospy.Duration(0.2)
        traj.points = [pt]
        pub_arm.publish(traj)
        # 夹爪轨迹
        traj_g = JointTrajectory()
        traj_g.header.stamp = rospy.Time.now()
        traj_g.joint_names = [GRIPPER_JOINT]
        ptg = JointTrajectoryPoint()
        ptg.positions = [math.radians(grip_deg)]
        ptg.time_from_start = rospy.Duration(0.2)
        traj_g.points = [ptg]
        pub_gripper.publish(traj_g)
        
        # 更新状态
        last_angles = arm_deg.copy()
        last_gripper_angle = grip_deg
        last_command_time = time.time()
        stats['commands_sent'] += 1
        
        rospy.logdebug(f"[Gazebo] 发布: 臂{[round(a,1) for a in arm_deg]}, 夹爪{grip_deg:.1f}°")
        
    except Exception as e:
        rospy.logwarn(f"[Gazebo] 发布失败: {e}")
        stats['errors'] += 1

def initialize_pro450():
    """初始化Pro450网络连接"""
    global mc
    
    try:
        rospy.loginfo(f"[slider_control] 正在连接 Pro450 @ {PRO450_IP}:{PRO450_PORT}...")
        mc = Pro450Client(PRO450_IP, PRO450_PORT)
        time.sleep(1.0)
        
        # 上电
        rospy.loginfo("[slider_control] ⚡ 机械臂上电中...")
        mc.power_on()
        time.sleep(1.0)
        rospy.loginfo("[slider_control] ✅ 机械臂已上电")
        
        # 设置舵机校准
        rospy.loginfo("[slider_control] 🔧 设置舵机校准...")
        mc.set_servo_calibration(6)
        time.sleep(0.5)
        
        # 测试连接
        current_angles = mc.get_angles()
        rospy.loginfo(f"[slider_control] ✅ Pro450连接成功!")
        rospy.loginfo(f"[slider_control] 当前角度: {current_angles}")
        
        # 初始化夹爪
        gripper_ok = initialize_gripper()
        if gripper_ok:
            rospy.loginfo("[slider_control] ✅ 夹爪初始化成功")
            
            # 测试夹爪控制
            rospy.loginfo("[slider_control] 🧪 测试夹爪控制...")
            test_success = set_gripper_angle_pro450(0.0)  # 测试中间位置
            if test_success:
                rospy.loginfo("[slider_control] ✅ 夹爪控制测试成功")
            else:
                rospy.logwarn("[slider_control] ⚠️  夹爪控制测试失败")
        else:
            rospy.logwarn("[slider_control] ⚠️  夹爪未检测到或初始化失败，将跳过夹爪控制")
        
        mc.release_all_servos()
        time.sleep(0.5)
        
        return True
        
    except Exception as e:
        rospy.logerr(f"[slider_control] ❌ Pro450初始化失败: {e}")
        rospy.logerr("[slider_control] 请检查:")
        rospy.logerr("[slider_control] 1. 机械臂是否正确连接到网络")
        rospy.logerr(f"[slider_control] 2. IP地址 {PRO450_IP} 是否正确")
        rospy.logerr("[slider_control] 3. 机械臂是否已开机并正常工作")
        rospy.logerr("[slider_control] 4. 防火墙是否阻止了连接")
        return False

def print_stats():
    """打印统计信息"""
    if stats['total_messages'] > 0:
        efficiency = (stats['commands_sent'] / stats['total_messages']) * 100
        rospy.loginfo(f"[slider_control] 📊 统计: 消息:{stats['total_messages']}, "
                      f"发送:{stats['commands_sent']}, 跳过:{stats['commands_skipped']}, "
                      f"超限:{stats['limit_violations']}, 错误:{stats['errors']}, "
                      f"效率:{efficiency:.1f}%")

def main():
    global mc, mode, pub_arm, pub_gripper, MOVEIT_COLLISION_CHECK
    
    rospy.init_node("slider_control_450", anonymous=True)
    
    # 模式选择
    print("\n" + "="*60)
    print(" MyCobot Pro 450 滑块控制器")
    print("="*60)
    print("选择控制模式:")
    print("  1: 滑块 → Gazebo仿真 (仅控制Gazebo)")
    print("  2: 滑块 → 真实 MyCobot Pro 450 (仅控制真实机械臂)")
    print("="*60)
    print("⚠️  注意: 两种模式都需要先启动slider.launch来显示滑块GUI")
    print("    roslaunch mycobotpro450_gazebo slider.launch")
    print("="*60)
    inp = input("请输入 1 或 2 (默认 2): ").strip()
    
    mode = 1 if inp == "1" else 2
    mode_name = "Gazebo仿真" if mode == 1 else "真实 Pro 450"
    
    rospy.loginfo(f"[slider_control]  控制模式: {mode_name}")
    rospy.loginfo(f"[slider_control]   配置: 角度阈值={ANGLE_THRESHOLD}°, "
                  f"最大频率={MAX_COMMAND_RATE}Hz, 队列大小={COMMAND_QUEUE_SIZE}")
    rospy.loginfo(f"[slider_control]   安全限制: 关节限制已启用, 夹爪{GRIPPER_LIMITS[0]}°~{GRIPPER_LIMITS[1]}°")
    
    # 初始化MoveIt碰撞检测
    if COLLISION_CHECK_ENABLED:
        rospy.loginfo("[slider_control]   正在初始化MoveIt碰撞检测...")
        if initialize_moveit_collision_checker():
            rospy.loginfo("[slider_control]   碰撞检测: MoveIt + 基础规则 (双重保护)")
        else:
            rospy.loginfo("[slider_control]   碰撞检测: 基础规则 (MoveIt不可用)")
    else:
        rospy.logwarn("[slider_control]   碰撞检测: 已禁用")
    
    # 模式1: Gazebo仿真 - 需要发布器
    if mode == 1:
        pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)
        rospy.loginfo("[slider_control]  ROS发布器初始化完成 (Gazebo模式)")
    
    # 模式2: 真实机械臂 - 需要连接Pro450
    if mode == 2:
        # 初始化真实机械臂
        if not initialize_pro450():
            rospy.logerr("[slider_control] ❌ Pro450初始化失败，退出")
            return
        
        # 启动命令执行线程
        executor_thread = threading.Thread(target=command_executor, daemon=True)
        executor_thread.start()
        rospy.loginfo("[slider_control] 🔄 异步命令执行器已启动")
        
        # 启动高度监控线程
        monitor_thread = threading.Thread(target=monitor_height, daemon=True)
        monitor_thread.start()
        rospy.loginfo("[slider_control] 📊 实时高度监控已启动")
    
    # 订阅关节状态 (来自slider GUI发布的 /joint_states)
    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    rospy.loginfo("[slider_control]  已订阅 /joint_states 话题 (等待滑块GUI输入)")
    
    # 订阅末端坐标 (来自coords_broadcaster发布的 /pro450/end_effector_coords)
    from geometry_msgs.msg import Point
    rospy.Subscriber("/pro450/end_effector_coords", Point, coords_callback, queue_size=1)
    rospy.loginfo("[slider_control]  已订阅 /pro450/end_effector_coords 话题 (末端坐标广播)")
    
    rospy.loginfo(f"[slider_control]  {mode_name}控制器启动成功，等待滑块输入...")
    rospy.loginfo("[slider_control]  Tips:")
    rospy.loginfo("[slider_control]    - 请确保已启动 slider.launch 来显示滑块GUI")
    rospy.loginfo("[slider_control]    - 角度超限时会自动限制并显示警告")
    rospy.loginfo("[slider_control]    - 碰撞检测会自动调整危险姿态")
    if MOVEIT_COLLISION_CHECK:
        rospy.loginfo("[slider_control]    - MoveIt碰撞检测已启用，检测自碰撞和环境碰撞")
        rospy.loginfo("[slider_control]    - 地面碰撞对象已添加，夹爪不会撞地")
    if mode == 1:
        rospy.loginfo("[slider_control]    - 模式1: 滑块控制Gazebo仿真机械臂")
    elif mode == 2:
        rospy.loginfo(f"[slider_control]    - 模式2: 滑块控制真实机械臂 {PRO450_IP}:{PRO450_PORT}")
        rospy.loginfo("[slider_control]    -  Gazebo机械臂不会移动，仅控制真实机械臂")
        rospy.loginfo("[slider_control]    - 夹爪角度自动映射: Gazebo(-60~60) → Pro450(0~100)")
    rospy.loginfo("[slider_control]    - 按 Ctrl+C 安全退出")
    
    # 定期打印统计信息
    def stats_timer():
        while not rospy.is_shutdown():
            time.sleep(15.0)  # 每15秒打印一次
            print_stats()
    
    stats_thread = threading.Thread(target=stats_timer, daemon=True)
    stats_thread.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[slider_control] 收到中断信号，正在关闭...")
    finally:
        print_stats()  # 最终统计
        if mc is not None:
            try:
                mc.release_all_servos()
                rospy.loginfo("[slider_control] ✅ 已释放所有舵机")
            except:
                pass
        rospy.loginfo("[slider_control] 程序已安全退出")

if __name__ == "__main__":
    main()

