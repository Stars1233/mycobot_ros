#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
angle_conversion_demo.py - 角度转换演示
展示 follow_display_gazebo.py 中的角度转换逻辑
"""

import math

def demonstrate_angle_conversion():
    """演示角度转换过程"""
    
    print("🔄 MyCobot Pro 450 角度转换逻辑演示")
    print("="*60)
    
    # 1. 机械臂关节角度转换
    print("\n1️⃣ 机械臂关节角度转换 (度 → 弧度)")
    print("-" * 40)
    
    # 模拟真实机械臂角度（度）
    real_arm_angles_deg = [0, 45, -30, 90, -45, 180]
    
    print("真实机械臂角度 (度):")
    for i, angle in enumerate(real_arm_angles_deg):
        print(f"  joint{i+1}: {angle:6.1f}°")
    
    # 转换为弧度（Gazebo使用）
    gazebo_arm_angles_rad = [math.radians(a) for a in real_arm_angles_deg]
    
    print("\nGazebo仿真角度 (弧度):")
    for i, angle in enumerate(gazebo_arm_angles_rad):
        print(f"  joint{i+1}: {angle:6.3f} rad")
    
    print("\n转换公式: 弧度 = 度 × π/180")
    
    # 2. 夹爪角度映射
    print("\n2️⃣ 夹爪角度映射 (Pro450范围 → Gazebo范围)")
    print("-" * 50)
    
    # Pro450夹爪参数
    PRO450_MIN = 0
    PRO450_MAX = 100
    GAZEBO_MIN = 0
    GAZEBO_MAX = 57.3
    
    print(f"Pro450夹爪范围: {PRO450_MIN} - {PRO450_MAX}")
    print(f"Gazebo夹爪范围: {GAZEBO_MIN} - {GAZEBO_MAX}°")
    
    # 测试不同的Pro450夹爪角度
    test_pro450_angles = [0, 25, 50, 75, 100]
    
    print("\n夹爪角度映射示例:")
    print("Pro450角度 → Gazebo角度")
    
    for pro450_angle in test_pro450_angles:
        # 线性映射公式
        gazebo_angle = ((pro450_angle - PRO450_MIN) / 
                       (PRO450_MAX - PRO450_MIN)) * \
                      (GAZEBO_MAX - GAZEBO_MIN) + GAZEBO_MIN
        
        print(f"  {pro450_angle:3d}°      → {gazebo_angle:5.1f}°")
    
    print(f"\n映射公式:")
    print(f"gazebo_angle = ((pro450_angle - {PRO450_MIN}) / ({PRO450_MAX} - {PRO450_MIN})) × ({GAZEBO_MAX} - {GAZEBO_MIN}) + {GAZEBO_MIN}")
    
    # 3. 完整转换示例
    print("\n3️⃣ 完整转换示例")
    print("-" * 30)
    
    # 模拟一个完整的机械臂状态
    real_angles = [30, -45, 60, -90, 45, 0]  # 度
    real_gripper = 75  # Pro450夹爪角度
    
    print("真实机械臂状态:")
    print(f"  关节角度: {real_angles} (度)")
    print(f"  夹爪角度: {real_gripper} (Pro450范围)")
    
    # 转换为Gazebo格式
    gazebo_angles = [math.radians(a) for a in real_angles]
    gazebo_gripper = ((real_gripper - PRO450_MIN) / 
                     (PRO450_MAX - PRO450_MIN)) * \
                    (GAZEBO_MAX - GAZEBO_MIN) + GAZEBO_MIN
    gazebo_gripper_rad = math.radians(gazebo_gripper)
    
    print("\nGazebo仿真状态:")
    print(f"  关节角度: {[round(a, 3) for a in gazebo_angles]} (弧度)")
    print(f"  夹爪角度: {gazebo_gripper:.1f}° = {gazebo_gripper_rad:.3f} rad")
    
    # 4. 数据验证
    print("\n4️⃣ 数据验证函数")
    print("-" * 25)
    
    def is_valid_angles(angles):
        """检查关节角度是否有效"""
        if angles is None or len(angles) != 6:
            return False
        for a in angles:
            if not isinstance(a, (int, float)) or a < -180 or a > 180:
                return False
        return True
    
    def is_valid_gripper_angle(angle):
        """检查夹爪角度是否有效"""
        if angle is None:
            return False
        if isinstance(angle, (int, float)) and 0 <= angle <= 100:
            return True
        return False
    
    # 测试验证函数
    test_cases = [
        ([0, 45, -30, 90, -45, 180], "有效关节角度"),
        ([0, 45, -30, 90, -45, 200], "无效关节角度(超范围)"),
        ([0, 45, -30, 90, -45], "无效关节角度(数量不足)"),
        (None, "空值")
    ]
    
    print("关节角度验证测试:")
    for angles, desc in test_cases:
        valid = is_valid_angles(angles)
        print(f"  {desc}: {'✅' if valid else '❌'}")
    
    gripper_test_cases = [
        (50, "有效夹爪角度"),
        (150, "无效夹爪角度(超范围)"),
        (-10, "无效夹爪角度(负值)"),
        (None, "空值")
    ]
    
    print("\n夹爪角度验证测试:")
    for angle, desc in gripper_test_cases:
        valid = is_valid_gripper_angle(angle)
        print(f"  {desc}: {'✅' if valid else '❌'}")

def interactive_converter():
    """交互式角度转换器"""
    print("\n🎮 交互式角度转换器")
    print("="*40)
    
    while True:
        try:
            print("\n选择转换类型:")
            print("1. 关节角度 (度 → 弧度)")
            print("2. 夹爪角度 (Pro450 → Gazebo)")
            print("3. 退出")
            
            choice = input("\n请选择 (1-3): ").strip()
            
            if choice == '1':
                angle_deg = float(input("输入关节角度 (度): "))
                angle_rad = math.radians(angle_deg)
                print(f"结果: {angle_deg}° = {angle_rad:.4f} 弧度")
                
            elif choice == '2':
                pro450_angle = float(input("输入Pro450夹爪角度 (0-100): "))
                if 0 <= pro450_angle <= 100:
                    gazebo_angle = (pro450_angle / 100) * 57.3
                    gazebo_rad = math.radians(gazebo_angle)
                    print(f"结果: Pro450 {pro450_angle}° = Gazebo {gazebo_angle:.1f}° = {gazebo_rad:.4f} 弧度")
                else:
                    print("❌ 角度超出范围 (0-100)")
                    
            elif choice == '3':
                print("👋 退出转换器")
                break
                
            else:
                print("❌ 无效选择")
                
        except ValueError:
            print("❌ 请输入有效数字")
        except KeyboardInterrupt:
            print("\n👋 退出转换器")
            break

if __name__ == "__main__":
    # 运行演示
    demonstrate_angle_conversion()
    
    # 交互式转换器
    try:
        interactive_converter()
    except KeyboardInterrupt:
        print("\n👋 程序退出")