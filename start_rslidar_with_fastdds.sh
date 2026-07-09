#!/bin/bash
###############################################################################
# 雷达驱动启动脚本 - 启用 FastDDS 共享内存优化
# 用途：解决点云传输导致的帧率下降和 std dev 增大问题
###############################################################################

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  FastDDS 共享内存优化 - 雷达驱动启动  ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 设置 FastDDS 配置文件路径
FASTDDS_CONFIG="$HOME/.config/fastdds_shm.xml"

# 检查配置文件是否存在
if [ ! -f "$FASTDDS_CONFIG" ]; then
    echo -e "${RED}错误：FastDDS 配置文件不存在！${NC}"
    echo -e "${YELLOW}请先创建配置文件：$FASTDDS_CONFIG${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} 找到 FastDDS 配置文件：$FASTDDS_CONFIG"
echo ""

# 设置环境变量
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_CONFIG
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

echo -e "${YELLOW}环境变量设置：${NC}"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "  RMW_FASTRTPS_USE_QOS_FROM_XML=$RMW_FASTRTPS_USE_QOS_FROM_XML"
echo ""

# 获取 ROS2 工作空间路径
WORKSPACE="$HOME/humanoid_ws"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}错误：工作空间不存在！${NC}"
    echo -e "${YELLOW}请确认工作空间路径：$WORKSPACE${NC}"
    exit 1
fi

# Source 工作空间
echo -e "${GREEN}正在 Source 工作空间...${NC}"
source "$WORKSPACE/install/setup.bash"

# 启动雷达驱动
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  启动雷达驱动节点                    ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}提示：启动后可以运行以下命令验证效果：${NC}"
echo -e "  ${GREEN}ros2 topic hz /airy_points${NC}"
echo ""

# 启动节点
ros2 launch rslidar_sdk start.py
