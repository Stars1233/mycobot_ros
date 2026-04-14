#!/bin/bash
# check_joints.sh - 快速查看MyCobot Pro 450关节角度的脚本

echo "🤖 MyCobot Pro 450 关节角度查看工具"
echo "=================================="

# 检查ROS是否运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ ROS核心未运行，请先启动 roscore"
    exit 1
fi

# 检查Gazebo是否运行
if ! pgrep -x "gzserver" > /dev/null; then
    echo "⚠️  Gazebo仿真未运行，请先启动Gazebo"
    echo "   使用命令: roslaunch mycobotpro450_gazebo gazebo.launch"
    exit 1
fi

echo "📡 正在获取关节状态..."
echo ""

# 方法1: 使用rostopic echo (显示一次)
echo "方法1: 使用 rostopic echo"
echo "------------------------"
timeout 3s rostopic echo /joint_states -n 1 | grep -E "(name|position)" | head -20

echo ""
echo "方法2: 使用 rostopic hz (显示频率)"
echo "--------------------------------"
echo "关节状态发布频率:"
timeout 3s rostopic hz /joint_states

echo ""
echo "方法3: 可用的ROS话题"
echo "-------------------"
echo "与关节相关的话题:"
rostopic list | grep -E "(joint|arm|gripper)"

echo ""
echo "💡 提示:"
echo "  - 使用 'rostopic echo /joint_states' 持续监控"
echo "  - 使用 'rqt_plot /joint_states/position[0]' 绘制关节1角度曲线"
echo "  - 使用 'rqt_joint_trajectory_controller' 图形化控制"
echo "  - 角度单位为弧度，需要乘以 180/π 转换为度"