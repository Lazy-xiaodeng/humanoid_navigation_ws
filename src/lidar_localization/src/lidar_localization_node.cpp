/**
 * @file lidar_localization_node.cpp
 * @brief 激光雷达定位ROS2节点主程序
 *
 * 本文件是ROS2节点的入口程序，负责：
 * 1. 初始化ROS2通信
 * 2. 创建PCLLocalization组件实例
 * 3. 启动多线程执行器处理回调（提高性能）
 */

#include <lidar_localization/lidar_localization_component.hpp>

/**
 * @brief 主函数，程序入口
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码
 */
int main(int argc, char * argv[])
{
  // 初始化ROS2通信
  rclcpp::init(argc, argv);

  // 创建多线程执行器（使用4个线程处理回调，提高点云处理性能）
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);

  // 创建节点选项（使用默认选项）
  rclcpp::NodeOptions options;

  // 创建PCLLocalization组件实例
  std::shared_ptr<PCLLocalization> pcl_l = std::make_shared<PCLLocalization>(options);

  // 将节点添加到执行器
  executor.add_node(pcl_l->get_node_base_interface());

  // 开始处理回调（阻塞直到节点关闭）
  executor.spin();

  // 关闭ROS2通信
  rclcpp::shutdown();

  return 0;
}
