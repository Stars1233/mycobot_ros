#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
coords_broadcaster.py - MyCobot Pro 450 末端坐标广播服务
功能：高频读取真实机械臂末端坐标，通过ROS话题广播
用途：其他节点可以订阅末端坐标，用于碰撞检测等
"""

import time
import rospy
from geometry_msgs.msg import Point
from pymycobot import Pro450Client

# Pro450 连接参数
PRO450_IP = "192.168.0.232"
PRO450_PORT = 4500

# 全局变量
mc = None
pub_coords = None

# 广播频率
BROADCAST_RATE = 50  # Hz

def initialize_pro450():
    """初始化Pro450网络连接"""
    global mc
    
    try:
        rospy.loginfo(f"[coords_broadcaster] 正在连接 Pro450 @ {PRO450_IP}:{PRO450_PORT}...")
        mc = Pro450Client(PRO450_IP, PRO450_PORT)
        time.sleep(1.0)
        
        # 测试连接
        current_angles = mc.get_angles()
        rospy.loginfo(f"[coords_broadcaster] ✅ Pro450连接成功!")
        rospy.loginfo(f"[coords_broadcaster] 当前角度: {current_angles}")
        
        return True
        
    except Exception as e:
        rospy.logerr(f"[coords_broadcaster] ❌ Pro450初始化失败: {e}")
        return False

def broadcast_coords():
    """高频广播末端坐标"""
    global mc, pub_coords
    
    rate = rospy.Rate(BROADCAST_RATE)
    
    while not rospy.is_shutdown():
        try:
            # 获取末端坐标
            coords = mc.get_coords()
            
            if coords is not None and len(coords) >= 3:
                # 创建Point消息 (x, y, z)
                point = Point()
                point.x = coords[0]  # X坐标 (mm)
                point.y = coords[1]  # Y坐标 (mm)
                point.z = coords[2]  # Z坐标 (高度, mm)
                
                # 发布
                pub_coords.publish(point)
                
                rospy.logdebug(f"[coords_broadcaster] 末端坐标: X={coords[0]:.1f}mm, Y={coords[1]:.1f}mm, Z={coords[2]:.1f}mm")
            else:
                rospy.logwarn_throttle(5, "[coords_broadcaster] 获取末端坐标失败")
                
        except Exception as e:
            rospy.logerr_throttle(5, f"[coords_broadcaster] 广播错误: {e}")
        
        rate.sleep()

def main():
    """主函数"""
    global mc, pub_coords
    
    rospy.init_node("coords_broadcaster", anonymous=True)
    
    print("\n" + "="*60)
    print("🤖 MyCobot Pro 450 末端坐标广播服务")
    print("="*60)
    print("功能: 高频读取真实机械臂末端坐标，通过ROS话题广播")
    print("话题: /pro450/end_effector_coords (geometry_msgs/Point)")
    print("      - x: X坐标 (mm)")
    print("      - y: Y坐标 (mm)")
    print("      - z: Z坐标/高度 (mm)")
    print("="*60 + "\n")
    
    # 初始化 Pro450 连接
    if not initialize_pro450():
        rospy.logerr("[main] ❌ Pro450初始化失败，退出")
        return
    
    # 初始化ROS发布者
    pub_coords = rospy.Publisher("/pro450/end_effector_coords", Point, queue_size=10)
    rospy.loginfo("[main] ROS发布器初始化完成")
    rospy.loginfo(f"[main] 广播频率: {BROADCAST_RATE}Hz")
    rospy.loginfo("[main] 末端坐标广播服务已启动")
    rospy.loginfo("[main] 💡 按 Ctrl+C 安全退出")
    
    # 注册退出处理函数
    def cleanup():
        if mc is not None:
            try:
                rospy.loginfo("[main] 程序退出...")
            except:
                pass
    
    rospy.on_shutdown(cleanup)
    
    # 启动广播
    broadcast_coords()
    
    rospy.loginfo("[main] 👋 程序已安全退出")

if __name__ == "__main__":
    main()

