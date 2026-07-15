// Offline adapter around the official SOLiD RAH descriptor (BSD-3-Clause).
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Config {
  int angles = 60, ranges = 40, heights = 32;
  double min_distance = 0.8, max_distance = 20.0;
  double fov_down = -35.0, fov_up = 35.0, voxel = 0.4, sensor_height = 1.215;
};

Eigen::VectorXd make_solid(const std::string &path, const Config &cfg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(path, *raw) < 0) throw std::runtime_error("cannot read " + path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto point : raw->points) {
    point.z -= cfg.sensor_height;
    const double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (distance > cfg.min_distance && distance < cfg.max_distance) cropped->push_back(point);
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cropped);
  filter.setLeafSize(cfg.voxel, cfg.voxel, cfg.voxel);
  filter.filter(cloud);

  Eigen::MatrixXd range = Eigen::MatrixXd::Zero(cfg.ranges, cfg.heights);
  Eigen::MatrixXd angle = Eigen::MatrixXd::Zero(cfg.angles, cfg.heights);
  const double angle_gap = 360.0 / cfg.angles;
  const double range_gap = cfg.max_distance / cfg.ranges;
  const double height_gap = (cfg.fov_up - cfg.fov_down) / cfg.heights;
  for (const auto &point : cloud.points) {
    double theta = std::atan2(point.y, point.x) * 180.0 / M_PI;
    if (theta < 0.0) theta += 360.0;
    const double planar = std::hypot(point.x, point.y);
    const double elevation = std::atan2(point.z, planar) * 180.0 / M_PI;
    const int ri = std::clamp(static_cast<int>(planar / range_gap), 0, cfg.ranges - 1);
    const int ai = std::clamp(static_cast<int>(theta / angle_gap), 0, cfg.angles - 1);
    const int hi = std::clamp(static_cast<int>((elevation - cfg.fov_down) / height_gap), 0, cfg.heights - 1);
    range(ri, hi) += 1.0;
    angle(ai, hi) += 1.0;
  }
  Eigen::VectorXd weights(cfg.heights);
  for (int col = 0; col < cfg.heights; ++col) weights(col) = range.col(col).sum();
  const double span = weights.maxCoeff() - weights.minCoeff();
  if (span > 0.0) {
    weights = (weights.array() - weights.minCoeff()) / span;
  } else {
    weights = Eigen::VectorXd::Ones(cfg.heights);
  }
  Eigen::VectorXd descriptor(cfg.ranges + cfg.angles);
  descriptor.head(cfg.ranges) = range * weights;
  descriptor.tail(cfg.angles) = angle * weights;
  return descriptor;
}

int main(int argc, char **argv) {
  if (argc != 3 && argc != 9) {
    std::cerr << "usage: solid_official_descriptor_cli manifest.txt descriptors.csv [max_range voxel fov_down fov_up sensor_height min_range]\n";
    return 2;
  }
  Config cfg;
  if (argc == 9) {
    cfg.max_distance = std::stod(argv[3]);
    cfg.voxel = std::stod(argv[4]);
    cfg.fov_down = std::stod(argv[5]);
    cfg.fov_up = std::stod(argv[6]);
    cfg.sensor_height = std::stod(argv[7]);
    cfg.min_distance = std::stod(argv[8]);
  }
  std::ifstream manifest(argv[1]);
  std::ofstream output(argv[2]);
  output << std::setprecision(17);
  std::string line;
  while (std::getline(manifest, line)) {
    const auto split = line.find('\t');
    if (split == std::string::npos) continue;
    const std::string id = line.substr(0, split);
    const Eigen::VectorXd descriptor = make_solid(line.substr(split + 1), cfg);
    output << id;
    for (int i = 0; i < descriptor.size(); ++i) output << ',' << descriptor(i);
    output << '\n';
  }
  return 0;
}
