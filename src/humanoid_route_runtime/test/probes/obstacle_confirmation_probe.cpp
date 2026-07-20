/* 验证低速停滞只有在 ROI/costmap 确认障碍后才进入障碍等待。 */

#include <iostream>

#include "humanoid_route_runtime/obstacle_wait_manager.hpp"

using humanoid_route_runtime::EnvironmentRuntimeState;
using humanoid_route_runtime::ObstacleWaitManager;
using humanoid_route_runtime::RouteRuntimeConfig;

int main()
{
  ObstacleWaitManager manager;
  RouteRuntimeConfig config;
  config.obstacle_block_require_sensor_confirmation = true;
  config.obstacle_block_sensor_timeout_sec = 1.0;

  EnvironmentRuntimeState environment;
  environment.has_current_pose = true;
  environment.latest_roi_stamp = 100.0;
  environment.latest_roi_has_obstacle = false;
  environment.latest_costmap_stamp = 100.0;
  environment.latest_costmap_width = 10;
  environment.latest_costmap_height = 10;
  environment.latest_costmap_resolution = 0.1;
  environment.latest_costmap_data.assign(100, 0);

  const auto clear = manager.confirm_obstacle(100.5, config, environment);
  if (clear.confirmed) {
    std::cerr << "clear sensors must not confirm a low-speed obstacle" << std::endl;
    return 1;
  }

  environment.latest_roi_has_obstacle = true;
  const auto roi = manager.confirm_obstacle(100.5, config, environment);
  if (!roi.confirmed || !roi.roi_blocked) {
    std::cerr << "fresh ROI obstacle must confirm blockage" << std::endl;
    return 2;
  }

  environment.latest_roi_has_obstacle = false;
  environment.latest_costmap_data[2] = 100;
  const auto costmap = manager.confirm_obstacle(100.5, config, environment);
  if (!costmap.confirmed || !costmap.costmap_blocked) {
    std::cerr << "front costmap obstacle must confirm blockage" << std::endl;
    return 3;
  }

  const auto stale = manager.confirm_obstacle(102.0, config, environment);
  if (stale.confirmed) {
    std::cerr << "stale sensors must not confirm blockage" << std::endl;
    return 4;
  }

  config.obstacle_block_require_sensor_confirmation = false;
  const auto legacy = manager.confirm_obstacle(102.0, config, environment);
  if (!legacy.confirmed || legacy.reason != "legacy_speed_only") {
    std::cerr << "rollback gate must preserve legacy speed-only behavior" << std::endl;
    return 5;
  }

  std::cout << "obstacle_confirmation_probe: PASS" << std::endl;
  return 0;
}
