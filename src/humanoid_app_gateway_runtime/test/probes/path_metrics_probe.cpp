/*
 * path_metrics_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期路径指标验证工具，构造多组固定路径并输出 C++ 分析结果。
 * 2. 验证路径长度、预计耗时、平滑度、复杂度、安全等级、转弯计数等字段。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证路径分析结果是否符合线上协议公式。
 */

#include <iostream>
#include <string>
#include <vector>

#include "humanoid_app_gateway_runtime/path_metrics.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

void add_string(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::string & value)
{
  object.AddMember(
    rapidjson::Value(key, allocator).Move(),
    rapidjson::Value(value.c_str(), allocator).Move(),
    allocator);
}

void add_result(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const humanoid_app_gateway_runtime::PathAnalysisResult & result,
  const double estimated_duration)
{
  rapidjson::Value value(rapidjson::kObjectType);
  add_string(value, allocator, "smoothness", result.smoothness);
  add_string(value, allocator, "safety_level", result.safety_level);
  add_string(value, allocator, "complexity", result.complexity);
  value.AddMember("turn_count", result.turn_count, allocator);
  value.AddMember("total_distance", result.total_distance, allocator);
  value.AddMember("avg_segment_length", result.avg_segment_length, allocator);
  value.AddMember("max_angle_change", result.max_angle_change_deg, allocator);
  value.AddMember("segment_count", result.segment_count, allocator);
  value.AddMember("estimated_duration", estimated_duration, allocator);
  object.AddMember(rapidjson::Value(key, allocator).Move(), value, allocator);
}

}  // namespace

int main()
{
  humanoid_app_gateway_runtime::PathMetrics metrics;

  const std::vector<humanoid_app_gateway_runtime::PathPoint2D> straight{
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  const std::vector<humanoid_app_gateway_runtime::PathPoint2D> turn{
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}};
  const std::vector<humanoid_app_gateway_runtime::PathPoint2D> single{{0.0, 0.0}};

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  const auto straight_result = metrics.analyze_path_properties(straight);
  add_result(
    document,
    allocator,
    "straight",
    straight_result,
    metrics.estimate_path_duration(straight_result.total_distance));

  const auto turn_result = metrics.analyze_path_properties(turn);
  add_result(
    document,
    allocator,
    "turn",
    turn_result,
    metrics.estimate_path_duration(turn_result.total_distance));

  const auto single_result = metrics.analyze_path_properties(single);
  add_result(
    document,
    allocator,
    "single",
    single_result,
    metrics.estimate_path_duration(single_result.total_distance));

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
