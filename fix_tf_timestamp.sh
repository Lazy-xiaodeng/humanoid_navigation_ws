#!/bin/bash
# 修复TF时间戳同步问题的综合脚本

echo "========================================="
echo "  人形导航 TF 时间戳同步修复脚本"
echo "========================================="

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}[1/5] 备份原始配置文件...${NC}"
BACKUP_DIR="/home/ubuntu/humanoid_ws/backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

cp /home/ubuntu/humanoid_ws/src/humanoid_navigation2/config/nav2_params.yaml "$BACKUP_DIR/"
cp /home/ubuntu/humanoid_ws/src/humanoid_navigation2/config/slam_toolbox_params.yaml "$BACKUP_DIR/"
cp /home/ubuntu/humanoid_ws/src/fast_lio_robosense/config/robosenseAiry.yaml "$BACKUP_DIR/"
cp /home/ubuntu/humanoid_ws/src/humanoid_bringup/launch/robot_real.launch.py "$BACKUP_DIR/"

echo -e "${GREEN}[2/5] 修复 fastlio_mapping 的 TF 时间戳...${NC}"
echo "  说明: fastlio使用点云结束时间,需要改为使用ROS系统时间"

FASTLIO_SRC="/home/ubuntu/humanoid_ws/src/fast_lio_robosense/src/laserMapping.cpp"
if [ -f "$FASTLIO_SRC" ]; then
    # 修改 publish_odometry 函数中的时间戳
    # 将 get_ros_time(lidar_end_time) 改为 this->now() 或 rclcpp::Clock().now()
    sed -i 's/trans\.header\.stamp = get_ros_time(lidar_end_time);/trans.header.stamp = rclcpp::Clock().now();/' "$FASTLIO_SRC"
    sed -i 's/odomAftMapped\.header\.stamp = get_ros_time(lidar_end_time);/odomAftMapped.header.stamp = rclcpp::Clock().now();/' "$FASTLIO_SRC"
    echo -e "${GREEN}  ✓ fastlio TF时间戳已修复${NC}"
else
    echo -e "${RED}  ✗ fastlio源文件未找到: $FASTLIO_SRC${NC}"
fi

echo -e "${GREEN}[3/5] 优化 nav2_params.yaml 的 TF 容忍度...${NC}"
NAV2_PARAMS="/home/ubuntu/humanoid_ws/src/humanoid_navigation2/config/nav2_params.yaml"
if [ -f "$NAV2_PARAMS" ]; then
    # 增加transform_tolerance
    sed -i 's/local_costmap:\n    ros__parameters:\n      transform_tolerance: 0.5/local_costmap:\n    ros__parameters:\n      transform_tolerance: 3.0/' "$NAV2_PARAMS"
    sed -i 's/global_costmap:\n  global_costmap:\n    ros__parameters:\n      transform_tolerance: 0.5/global_costmap:\n  global_costmap:\n    ros__parameters:\n      transform_tolerance: 3.0/' "$NAV2_PARAMS"
    echo -e "${GREEN}  ✓ costmap TF容忍度已增加到3.0秒${NC}"
else
    echo -e "${RED}  ✗ nav2参数文件未找到${NC}"
fi

echo -e "${GREEN}[4/5] 优化 slam_toolbox TF 配置...${NC}"
SLAM_PARAMS="/home/ubuntu/humanoid_ws/src/humanoid_navigation2/config/slam_toolbox_params.yaml"
if [ -f "$SLAM_PARAMS" ]; then
    # 确保restamp_tf为true
    sed -i 's/restamp_tf:.*/restamp_tf: true/' "$SLAM_PARAMS"
    echo -e "${GREEN}  ✓ slam_toolbox TF配置已优化${NC}"
else
    echo -e "${RED}  ✗ slam_toolbox参数文件未找到${NC}"
fi

echo -e "${GREEN}[5/5] 编译工作区...${NC}"
cd /home/ubuntu/humanoid_ws
colcon build --packages-select fast_lio_robosense humanoid_navigation2 humanoid_relocalization --symlink-install

echo ""
echo -e "${YELLOW}=========================================${NC}"
echo -e "${YELLOW}  修复完成! 需要执行以下步骤:${NC}"
echo -e "${YELLOW}=========================================${NC}"
echo ""
echo "1. 重新编译 fast_lio_robosense:"
echo "   cd /home/ubuntu/humanoid_ws && colcon build --packages-select fast_lio_robosense"
echo ""
echo "2. 重启导航栈:"
echo "   ros2 launch humanoid_bringup robot_real.launch.py"
echo ""
echo "3. 监控TF时间戳:"
echo "   ros2 run tf2_tools view_frames"
echo ""
echo "4. 检查TF变换是否正常:"
echo "   ros2 tf2_echo odom base_footprint"
echo ""
echo -e "${GREEN}备份已保存到: $BACKUP_DIR${NC}"
