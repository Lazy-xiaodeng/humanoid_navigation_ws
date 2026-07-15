/*
 * solid_database_builder_main.cpp
 *
 * 文件作用：
 *   1. 从建图关键帧清单读取 map 位姿和局部 PCD。
 *   2. 使用运行配置中的同一组 SOLiD 参数生成描述子与精配准点云数据库。
 *   3. 为运行节点的 solid_database_path 提供可重复生成的正式数据库。
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"
#include "humanoid_global_relocalization_runtime/solid.hpp"

namespace
{

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    while (!field.empty() && (field.back() == '\r' || field.back() == '\n')) {
      field.pop_back();
    }
    fields.push_back(field);
  }
  return fields;
}

Eigen::Matrix4d pose(double x, double y, double z, double yaw_deg)
{
  Eigen::Matrix4d value = Eigen::Matrix4d::Identity();
  const double yaw = yaw_deg * M_PI / 180.0;
  value(0, 0) = std::cos(yaw);
  value(0, 1) = -std::sin(yaw);
  value(1, 0) = std::sin(yaw);
  value(1, 1) = std::cos(yaw);
  value(0, 3) = x;
  value(1, 3) = y;
  value(2, 3) = z;
  return value;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 7 || std::string(argv[1]) != "--config" ||
    std::string(argv[3]) != "--keyframes" || std::string(argv[5]) != "--output")
  {
    std::cerr << "usage: solid_database_builder --config CONFIG --keyframes KEYFRAMES.csv --output DB.csv\n";
    return 2;
  }
  try {
    const auto config = humanoid_global_relocalization::load_config_file(argv[2]);
    std::ifstream input(argv[4]);
    if (!input) {
      throw std::runtime_error("failed to open keyframe manifest: " + std::string(argv[4]));
    }
    std::vector<humanoid_global_relocalization::SolidEntry> database;
    std::string line;
    bool header = true;
    while (std::getline(input, line)) {
      if (line.empty()) {
        continue;
      }
      if (header) {
        header = false;
        if (line.find("pcd") != std::string::npos) {
          continue;
        }
      }
      const auto fields = split_csv(line);
      // Supported manifest: frame_id,cloud_index,x,y,z,yaw_deg,pcd[,source_bag].
      if (fields.size() < 7) {
        throw std::runtime_error("invalid keyframe row: " + line);
      }
      humanoid_global_relocalization::SolidEntry entry;
      entry.cloud_index = std::stoi(fields[1]);
      entry.source_id = fields.size() > 7 ? fields[7] : "default";
      entry.map_to_base = pose(
        std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
      entry.cloud_path = fields[6];
      auto raw = humanoid_global_relocalization::load_pcd_xyz(entry.cloud_path);
      entry.descriptor = humanoid_global_relocalization::compute_solid_descriptor(raw, config.solid);
      entry.cloud = humanoid_global_relocalization::prepare_solid_registration_cloud(raw, config.solid);
      database.push_back(std::move(entry));
      if (database.size() % 50U == 0U) {
        std::cout << "[solid_database_builder] keyframes=" << database.size() << std::endl;
      }
    }
    humanoid_global_relocalization::save_solid_database(argv[6], database);
    std::cout << "[solid_database_builder] wrote entries=" << database.size() << " path=" << argv[6] << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "[solid_database_builder] failed: " << error.what() << '\n';
    return 1;
  }
}
