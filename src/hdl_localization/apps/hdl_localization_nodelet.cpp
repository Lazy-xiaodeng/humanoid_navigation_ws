// hdl localizaton ROS2 코드 1
#include <mutex>
#include <memory>
#include <iostream>
#include <chrono>
#include <future>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/msg/scan_matching_status.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

using namespace std;

namespace hdl_localization {

class HdlLocalizationNodelet : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet(const rclcpp::NodeOptions& options) : Node("hdl_localization", options) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    robot_odom_frame_id = declare_parameter<std::string>("robot_odom_frame_id", "robot_odom");
    odom_child_frame_id = declare_parameter<std::string>("odom_child_frame_id", "base_link");
    send_tf_transforms = declare_parameter<bool>("send_tf_transforms", true);
    cool_time_duration = declare_parameter<double>("cool_time_duration", 0.5);
    reg_method = declare_parameter<std::string>("reg_method", "NDT_OMP");
    ndt_neighbor_search_method = declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    ndt_neighbor_search_radius = declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    ndt_resolution = declare_parameter<double>("ndt_resolution", 1.0);
    enable_robot_odometry_prediction = declare_parameter<bool>("enable_robot_odometry_prediction", false);
    reject_scan_matching_without_convergence = declare_parameter<bool>("reject_scan_matching_without_convergence", true);
    max_scan_matching_fitness_score = declare_parameter<double>("max_scan_matching_fitness_score", 2.0);

    use_imu = declare_parameter<bool>("use_imu", true);
    invert_acc = declare_parameter<bool>("invert_acc", false);
    invert_gyro = declare_parameter<bool>("invert_gyro", false);
    if (use_imu) {
      RCLCPP_INFO(get_logger(), "enable imu-based prediction");
      imu_sub = create_subscription<sensor_msgs::msg::Imu>("/gpsimu_driver/imu_data", 256, std::bind(&HdlLocalizationNodelet::imu_callback, this, std::placeholders::_1));
    }
    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", 5, std::bind(&HdlLocalizationNodelet::points_callback, this, std::placeholders::_1));

    auto latch_qos = rclcpp::QoS(1).transient_local();
    globalmap_sub =
      create_subscription<sensor_msgs::msg::PointCloud2>("/globalmap", latch_qos, std::bind(&HdlLocalizationNodelet::globalmap_callback, this, std::placeholders::_1));

    initialpose_sub =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, std::bind(&HdlLocalizationNodelet::initialpose_callback, this, std::placeholders::_1));

    std::string odom_topic = declare_parameter<std::string>("odom_topic", "/hdl/odom");
    pose_pub = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
    aligned_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 5);
    status_pub = create_publisher<msg::ScanMatchingStatus>("/status", 5);

    // global localization
    use_global_localization = declare_parameter<bool>("use_global_localization", true);
    auto_relocalize_on_start = declare_parameter<bool>("auto_relocalize_on_start", false);
    auto_relocalize_after_rejections = declare_parameter<int>("auto_relocalize_after_rejections", 8);
    global_localization_pose_z_offset = declare_parameter<double>("global_localization_pose_z_offset", 0.0);
    validate_global_localization_with_scan_matching = declare_parameter<bool>("validate_global_localization_with_scan_matching", true);
    global_localization_max_fitness_score = declare_parameter<double>("global_localization_max_fitness_score", max_scan_matching_fitness_score);
    global_localization_max_candidates = declare_parameter<int>("global_localization_max_candidates", 10);
    global_localization_min_fitness_margin = declare_parameter<double>("global_localization_min_fitness_margin", 0.05);
    global_localization_required_consistent_results = declare_parameter<int>("global_localization_required_consistent_results", 1);
    global_localization_consistency_window = declare_parameter<int>("global_localization_consistency_window", 5);
    global_localization_consistency_xy_tolerance = declare_parameter<double>("global_localization_consistency_xy_tolerance", 0.8);
    global_localization_consistency_yaw_tolerance = declare_parameter<double>("global_localization_consistency_yaw_tolerance", 0.35);
    global_localization_query_accumulation_frames = declare_parameter<int>("global_localization_query_accumulation_frames", 1);
    global_localization_query_min_accumulation_frames = declare_parameter<int>("global_localization_query_min_accumulation_frames", 1);
    global_localization_post_accept_validation_frames = declare_parameter<int>("global_localization_post_accept_validation_frames", 0);
    global_localization_post_accept_max_rejections = declare_parameter<int>("global_localization_post_accept_max_rejections", 1);
    force_2d_pose = declare_parameter<bool>("force_2d_pose", true);
    force_2d_fixed_z = declare_parameter<bool>("force_2d_fixed_z", false);
    global_localization_use_height_filter = declare_parameter<bool>("global_localization_use_height_filter", false);
    global_localization_min_z = declare_parameter<double>("global_localization_min_z", 0.05);
    global_localization_max_z = declare_parameter<double>("global_localization_max_z", 1.9);
    global_localization_use_max_z_filter = declare_parameter<bool>("global_localization_use_max_z_filter", true);
    global_localization_query_timeout_sec = declare_parameter<double>("global_localization_query_timeout_sec", 60.0);
    globalmap_set_for_global_localization = false;
    auto_relocalize_done = false;
    relocalize_requested = false;
    consecutive_scan_matching_rejections = 0;
    global_pose_probation_frames_remaining = 0;
    global_pose_probation_rejections = 0;
    if (use_global_localization) {
      RCLCPP_INFO_STREAM(get_logger(), "wait for global localization services");
      global_localization_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      set_global_map_service = create_client<hdl_global_localization::srv::SetGlobalMap>(
        "/hdl_global_localization/set_global_map",
        rclcpp::ServicesQoS(),
        global_localization_callback_group);
      query_global_localization_service = create_client<hdl_global_localization::srv::QueryGlobalLocalization>(
        "/hdl_global_localization/query",
        rclcpp::ServicesQoS(),
        global_localization_callback_group);

      bool set_map_ready = false;
      for (int i = 0; i < 5; ++i) {
        if (set_global_map_service->wait_for_service(std::chrono::milliseconds(1000))) {
          set_map_ready = true;
          break;
        }
        RCLCPP_WARN(get_logger(), "Waiting for SetGlobalMap service");
        if (!rclcpp::ok()) {
          return;
        }
      }

      bool query_ready = false;
      if (set_map_ready) {
        for (int i = 0; i < 5; ++i) {
          if (query_global_localization_service->wait_for_service(std::chrono::milliseconds(1000))) {
            query_ready = true;
            break;
          }
          RCLCPP_WARN(get_logger(), "Waiting for QueryGlobalLocalization service");
          if (!rclcpp::ok()) {
            return;
          }
        }
      }

      if (!set_map_ready || !query_ready) {
        RCLCPP_ERROR(get_logger(), "global localization services are unavailable; continue with /initialpose local NDT mode");
        use_global_localization = false;
      }

      if (use_global_localization) {
        relocalize_server = create_service<std_srvs::srv::Empty>(
          "/relocalize",
          std::bind(&HdlLocalizationNodelet::relocalize, this, std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(),
          global_localization_callback_group);
        if (auto_relocalize_on_start) {
          auto_relocalize_timer = create_wall_timer(std::chrono::seconds(1), [this]() {
            const bool startup_relocalize = !auto_relocalize_done;
            const bool recovery_relocalize = relocalize_requested.exchange(false);
            if ((!startup_relocalize && !recovery_relocalize) || !globalmap || !last_scan || !globalmap_set_for_global_localization) {
              if (recovery_relocalize) {
                relocalize_requested = true;
              }
              return;
            }

            if (relocalizing) {
              return;
            }
            RCLCPP_INFO(get_logger(), recovery_relocalize ? "auto relocalize after scan matching rejection" : "auto relocalize on start");
            if (!perform_relocalize(false)) {
              RCLCPP_WARN(get_logger(), "auto relocalize failed; call /relocalize after the scan is stable or set /initialpose manually");
            }
          }, global_localization_callback_group);
        }
      }
    }
    initialize_params();
  }

private:
  struct PendingGlobalLocalizationCandidate {
    Eigen::Isometry3f pose;
  };

  rclcpp::Time node_time(const builtin_interfaces::msg::Time& stamp) const {
    return rclcpp::Time(stamp, get_clock()->get_clock_type());
  }

  rclcpp::Time zero_time() const {
    return rclcpp::Time((int64_t)0, get_clock()->get_clock_type());
  }

  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    if (reg_method == "NDT_OMP") {
      RCLCPP_INFO(get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(get_logger(), "invalid search method was given");
          RCLCPP_WARN(get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }

    // else if(reg_method.find("NDT_CUDA") != std::string::npos) {
    //   RCLCPP_INFO(get_logger(), "NDT_CUDA is selected");
    //   boost::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt(new fast_gicp::NDTCuda<PointT, PointT>);
    //   ndt->setResolution(ndt_resolution);

    //   if(reg_method.find("D2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
    //   } else if (reg_method.find("P2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    //   }

    //   if (ndt_neighbor_search_method == "DIRECT1") {
    //     RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    //   } else if (ndt_neighbor_search_method == "DIRECT7") {
    //     RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    //   } else if (ndt_neighbor_search_method == "DIRECT_RADIUS") {
    //     RCLCPP_INFO_STREAM(get_logger(), "search_method DIRECT_RADIUS is selected : " << ndt_neighbor_search_radius);
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
    //   } else {
    //     RCLCPP_WARN(get_logger(), "invalid search method was given");
    //   }
    //   return ndt;
    // }

    RCLCPP_ERROR_STREAM(get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    RCLCPP_INFO(get_logger(), "create registration method for localization");
    registration = create_registration();

    // global localization
    RCLCPP_INFO(get_logger(), "create registration method for fallback during relocalization");
    relocalizing = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    // initialize pose estimator
    bool specify_init_pose = declare_parameter<bool>("specify_init_pose", true);
    if (specify_init_pose) {
      RCLCPP_INFO(get_logger(), "initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration,
        get_clock()->now(),
        Eigen::Vector3f(declare_parameter<double>("init_pos_x", 0.0), declare_parameter<double>("init_pos_y", 0.0), declare_parameter<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(
          declare_parameter<double>("init_ori_w", 1.0),
          declare_parameter<double>("init_ori_x", 0.0),
          declare_parameter<double>("init_ori_y", 0.0),
          declare_parameter<double>("init_ori_z", 0.0)),
        cool_time_duration,
        force_2d_pose,
        force_2d_fixed_z ? global_localization_pose_z_offset : std::numeric_limits<double>::quiet_NaN()));
    }
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    // RCLCPP_INFO(get_logger(), "----------------");
    // RCLCPP_INFO(get_logger(), "imu_callback");
    // RCLCPP_INFO(get_logger(), "----------------");
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    RCLCPP_DEBUG(get_logger(), "points_callback");

    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    rclcpp::Time stamp = node_time(points_msg->header.stamp);
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    // point_msg의 sensor_msg/pointCloud2 type을 pcl_cloud type으로 형변환
    // sensor_msg/pointCloud2 -> pcl::PointCloud<PointT>
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    // Override PCL stamp with explicit-clock timestamp (pcl uses microseconds)
    pcl_cloud->header.stamp = stamp.nanoseconds() / 1000;

    if (pcl_cloud->empty()) {
      RCLCPP_ERROR(get_logger(), "cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud, *tf_buffer)) {
      RCLCPP_ERROR(get_logger(), "point cloud cannot be transformed into target frame!!");
      return;
    }

    auto filtered = downsample(cloud);
    last_scan = filtered;
    add_recent_scan(filtered);

    if (!pose_estimator) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "waiting for initial pose input or global relocalization!!");
      return;
    }

    if (!globalmap) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "globalmap has not been received!!");
      return;
    }

    if (relocalizing) {
      delta_estimater->add_frame(filtered);
    }

    Eigen::Matrix4f before = pose_estimator->matrix();

    // predict
    if (!use_imu) {
      pose_estimator->predict(stamp);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      // RCLCPP_INFO(get_logger(),"imu size is : %d ", imu_data.size());
      auto imu_iter = imu_data.begin();
      for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        rclcpp::Time imu_stamp = node_time((*imu_iter)->header.stamp);
        if (stamp < imu_stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double acc_sign = invert_acc ? -1.0 : 1.0;
        double gyro_sign = invert_gyro ? -1.0 : 1.0;
        pose_estimator->predict(imu_stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // odometry-based prediction
    rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
    if (enable_robot_odometry_prediction && last_correction_time != zero_time()) {
      geometry_msgs::msg::TransformStamped odom_delta;
      if (tf_buffer->canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration(std::chrono::milliseconds(100)))) {
        odom_delta =
          tf_buffer->lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration(std::chrono::milliseconds(0)));
      } else if (tf_buffer->canTransform(
                   odom_child_frame_id,
                   last_correction_time,
                   odom_child_frame_id,
                   zero_time(),
                   robot_odom_frame_id,
                   rclcpp::Duration(std::chrono::milliseconds(0)))) {
        odom_delta = tf_buffer->lookupTransform(
          odom_child_frame_id,
          last_correction_time,
          odom_child_frame_id,
          zero_time(),
          robot_odom_frame_id,
          rclcpp::Duration(std::chrono::milliseconds(0)));
      }

      if (odom_delta.header.stamp == zero_time()) {
        RCLCPP_WARN_STREAM(get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }

    // 문제가 되는 구문
    // correct
    bool correction_accepted = false;
    auto aligned = pose_estimator->correct(
      stamp,
      filtered,
      reject_scan_matching_without_convergence,
      max_scan_matching_fitness_score,
      &correction_accepted);
    if (!correction_accepted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "scan matching rejected: converged=%d fitness=%.3f threshold=%.3f",
        static_cast<int>(registration->hasConverged()),
        registration->getFitnessScore(),
        max_scan_matching_fitness_score);
      ++consecutive_scan_matching_rejections;
      if (
        use_global_localization &&
        auto_relocalize_after_rejections > 0 &&
        consecutive_scan_matching_rejections >= auto_relocalize_after_rejections &&
        !relocalizing) {
        consecutive_scan_matching_rejections = 0;
        relocalize_requested = true;
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "requesting global relocalization after repeated scan matching rejections");
      }
    } else {
      consecutive_scan_matching_rejections = 0;
    }

    if (!handle_global_pose_probation(correction_accepted)) {
      return;
    }

    if (aligned_pub->get_subscription_count()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_pub->publish(aligned_msg);
    }

    if (status_pub->get_subscription_count()) {
      publish_scan_matching_status(points_msg->header, aligned);
    }

    publish_odometry(stamp, pose_estimator->matrix());

    // RCLCPP_INFO(get_logger(), "");
    // RCLCPP_INFO(get_logger(), "----------finish points callback------------");
    // RCLCPP_INFO(get_logger(), "");
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    RCLCPP_INFO(get_logger(), "globalmap received!");

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    // pcl::PointCloud<PointT>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);

    if (use_global_localization) {
      RCLCPP_INFO(get_logger(), "set globalmap for global localization!");
      auto req = std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
      pcl::toROSMsg(*globalmap, req->global_map);
      globalmap_set_for_global_localization = false;
      set_global_map_service->async_send_request(
        req,
        [this](rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedFuture future) {
          try {
            future.get();
            globalmap_set_for_global_localization = true;
            RCLCPP_INFO(get_logger(), "globalmap set for global localization");
          } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(get_logger(), "Failed to call SetGlobalMap service: " << e.what());
          }
        });
    }
  }

  /**
   * @brief perform global localization to relocalize the sensor position
   * @param
   */
  bool relocalize(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    (void)req;
    (void)res;
    return perform_relocalize(true);
  }

  bool perform_relocalize(bool allow_overwrite_existing_pose) {
    pcl::PointCloud<PointT>::ConstPtr scan;
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex);

      if (relocalizing) {
        RCLCPP_WARN(get_logger(), "global relocalization is already running");
        return false;
      }

      if (last_scan == nullptr) {
        RCLCPP_INFO_STREAM(get_logger(), "no scan has been received");
        return false;
      }

      if (!globalmap_set_for_global_localization) {
        RCLCPP_WARN(get_logger(), "globalmap has not been set in hdl_global_localization yet");
        return false;
      }

      relocalizing = true;
      delta_estimater->reset();
      scan = build_global_localization_query();
      if (!scan || scan->empty()) {
        RCLCPP_WARN_STREAM(
          get_logger(),
          "not enough accumulated scans for global localization: have "
            << recent_scans.size()
            << " need " << std::max(1, global_localization_query_min_accumulation_frames));
        relocalizing = false;
        return false;
      }
    }
    if (global_localization_use_height_filter) {
      scan = filter_height(scan, global_localization_min_z, global_localization_max_z, global_localization_use_max_z_filter);
      if (scan->empty()) {
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "global localization scan is empty after height filter: z=["
            << global_localization_min_z << ", "
            << (global_localization_use_max_z_filter ? std::to_string(global_localization_max_z) : std::string("inf"))
            << "]");
        relocalizing = false;
        return false;
      }
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "global localization height-filtered scan points: " << scan->size()
          << " z=[" << global_localization_min_z << ", "
          << (global_localization_use_max_z_filter ? std::to_string(global_localization_max_z) : std::string("inf"))
          << "]");
    }

    auto query_req = std::make_shared<hdl_global_localization::srv::QueryGlobalLocalization::Request>();
    pcl::toROSMsg(*scan, query_req->cloud);
    query_req->max_num_candidates = std::max(1, global_localization_max_candidates);

    auto query_result_future = query_global_localization_service->async_send_request(query_req);
    auto query_timeout = std::chrono::duration<double>(global_localization_query_timeout_sec);
    if (query_result_future.wait_for(query_timeout) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Failed to call QueryGlobalLocalization service");
      relocalizing = false;
      return false;
    }
    auto query_result = query_result_future.get();

    if (query_result->poses.empty()) {
      RCLCPP_ERROR(get_logger(), "QueryGlobalLocalization returned empty poses array");
      relocalizing = false;
      return false;
    }

    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    if (!allow_overwrite_existing_pose && pose_estimator) {
      RCLCPP_WARN(get_logger(), "discard auto relocalization result because an initial pose is already available");
      auto_relocalize_done = true;
      relocalizing = false;
      return false;
    }

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    if (validate_global_localization_with_scan_matching) {
      struct ValidatedCandidate {
        size_t index;
        double fitness;
        double error;
        double inlier_fraction;
        Eigen::Isometry3f pose;
      };

      std::vector<ValidatedCandidate> accepted_candidates;
      registration->setInputSource(scan);

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Global localization candidates: returned=" << query_result->poses.size()
          << " requested=" << query_req->max_num_candidates
          << " fitness_threshold=" << global_localization_max_fitness_score
          << " min_margin=" << global_localization_min_fitness_margin
          << " query_points=" << scan->size());

      for (size_t i = 0; i < query_result->poses.size(); ++i) {
        const auto& result = query_result->poses[i];
        const double error = i < query_result->errors.size() ? query_result->errors[i] : std::numeric_limits<double>::quiet_NaN();
        const double inlier = i < query_result->inlier_fractions.size() ? query_result->inlier_fractions[i] : std::numeric_limits<double>::quiet_NaN();

        Eigen::Isometry3f candidate_pose = Eigen::Isometry3f::Identity();
        candidate_pose.linear() = Eigen::Quaternionf(
          result.orientation.w,
          result.orientation.x,
          result.orientation.y,
          result.orientation.z).toRotationMatrix();
        candidate_pose.translation() = Eigen::Vector3f(
          result.position.x,
          result.position.y,
          result.position.z + global_localization_pose_z_offset);
        candidate_pose = candidate_pose * delta_estimater->estimated_delta();
        apply_2d_constraints(candidate_pose);

        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
        registration->align(*aligned, candidate_pose.matrix());

        const bool converged = registration->hasConverged();
        const double fitness_score = registration->getFitnessScore();
        const bool fitness_score_valid = std::isfinite(fitness_score);

        RCLCPP_INFO_STREAM(
          get_logger(),
          "Global localization candidate[" << i << "]"
            << " trans=(" << result.position.x << ", " << result.position.y << ", " << result.position.z << ")"
            << " error=" << error
            << " inlier=" << inlier
            << " ndt_converged=" << converged
            << " ndt_fitness=" << fitness_score);

        if (!converged || !fitness_score_valid || fitness_score > global_localization_max_fitness_score) {
          continue;
        }

        Eigen::Isometry3f refined_pose(registration->getFinalTransformation());
        apply_2d_constraints(refined_pose);
        accepted_candidates.push_back(ValidatedCandidate{i, fitness_score, error, inlier, refined_pose});
      }

      if (accepted_candidates.empty()) {
        RCLCPP_WARN(get_logger(), "reject global localization result: no candidate passed NDT validation");
        relocalizing = false;
        return false;
      }

      std::sort(
        accepted_candidates.begin(),
        accepted_candidates.end(),
        [](const auto& lhs, const auto& rhs) {
          return lhs.fitness < rhs.fitness;
        });

      const auto& best = accepted_candidates[0];
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Best global localization candidate[" << best.index << "]"
          << " fitness=" << best.fitness
          << " error=" << best.error
          << " inlier=" << best.inlier_fraction
          << " accepted_candidates=" << accepted_candidates.size());

      if (accepted_candidates.size() >= 2) {
        const auto& second = accepted_candidates[1];
        const double margin = second.fitness - best.fitness;
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Second global localization candidate[" << second.index << "]"
            << " fitness=" << second.fitness
            << " margin=" << margin);

        if (margin < global_localization_min_fitness_margin) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "global localization result is ambiguous; requiring repeatability, fitness margin "
              << margin << " < " << global_localization_min_fitness_margin);
        }
      }

      pose = best.pose;
    } else {
      const auto& result = query_result->poses[0];

      RCLCPP_INFO_STREAM(get_logger(), "--- Global localization result ---");
      RCLCPP_INFO_STREAM(get_logger(), "Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
      RCLCPP_INFO_STREAM(get_logger(), "Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
      if (!query_result->errors.empty()) {
        RCLCPP_INFO_STREAM(get_logger(), "Error :" << query_result->errors[0]);
      }
      if (!query_result->inlier_fractions.empty()) {
        RCLCPP_INFO_STREAM(get_logger(), "Inlier:" << query_result->inlier_fractions[0]);
      }

      pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
      pose.translation() = Eigen::Vector3f(
        result.position.x,
        result.position.y,
        result.position.z + global_localization_pose_z_offset);
      pose = pose * delta_estimater->estimated_delta();
      apply_2d_constraints(pose);
    }

    const int consistent_count = update_global_localization_consistency(pose);
    if (consistent_count < std::max(1, global_localization_required_consistent_results)) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "hold global localization result until it is repeatable: consistent="
          << consistent_count << "/" << std::max(1, global_localization_required_consistent_results)
          << " window=" << global_localization_consistency_window
          << " xy_tol=" << global_localization_consistency_xy_tolerance
          << " yaw_tol=" << global_localization_consistency_yaw_tolerance);
      relocalizing = false;
      return false;
    }

    pose_estimator.reset(new hdl_localization::PoseEstimator(
      registration,
      get_clock()->now(),
      pose.translation(),
      Eigen::Quaternionf(pose.linear()),
      cool_time_duration,
      force_2d_pose,
      force_2d_fixed_z ? global_localization_pose_z_offset : std::numeric_limits<double>::quiet_NaN()));
    consecutive_scan_matching_rejections = 0;
    global_pose_probation_frames_remaining = std::max(0, global_localization_post_accept_validation_frames);
    global_pose_probation_rejections = 0;
    pending_global_localization_candidates.clear();
    auto_relocalize_done = true;

    relocalizing = false;

    return true;
  }

  pcl::PointCloud<PointT>::Ptr filter_height(
    const pcl::PointCloud<PointT>::ConstPtr& cloud,
    double min_z,
    double max_z,
    bool use_max_z) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());
    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.z)) {
        continue;
      }
      if (point.z >= min_z && (!use_max_z || point.z <= max_z)) {
        filtered->push_back(point);
      }
    }
    filtered->header = cloud->header;
    filtered->width = static_cast<uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = false;
    return filtered;
  }

  void add_recent_scan(const pcl::PointCloud<PointT>::ConstPtr& scan) {
    if (!scan || scan->empty()) {
      return;
    }

    recent_scans.push_back(scan);
    const size_t max_scans = static_cast<size_t>(std::max(1, global_localization_query_accumulation_frames));
    while (recent_scans.size() > max_scans) {
      recent_scans.pop_front();
    }
  }

  pcl::PointCloud<PointT>::ConstPtr build_global_localization_query() const {
    const size_t min_scans = static_cast<size_t>(std::max(1, global_localization_query_min_accumulation_frames));
    if (recent_scans.size() < min_scans) {
      return nullptr;
    }

    if (recent_scans.size() == 1) {
      return recent_scans.back();
    }

    pcl::PointCloud<PointT>::Ptr merged(new pcl::PointCloud<PointT>());
    for (const auto& scan : recent_scans) {
      *merged += *scan;
    }
    merged->header = recent_scans.back()->header;
    return downsample(merged);
  }

  static double normalize_angle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double pose_yaw(const Eigen::Isometry3f& pose) {
    return std::atan2(pose.matrix()(1, 0), pose.matrix()(0, 0));
  }

  bool poses_consistent(const Eigen::Isometry3f& lhs, const Eigen::Isometry3f& rhs) const {
    const Eigen::Vector2f lhs_xy(lhs.translation().x(), lhs.translation().y());
    const Eigen::Vector2f rhs_xy(rhs.translation().x(), rhs.translation().y());
    const double xy_distance = (lhs_xy - rhs_xy).norm();
    const double yaw_distance = std::abs(normalize_angle(pose_yaw(lhs) - pose_yaw(rhs)));
    return xy_distance <= global_localization_consistency_xy_tolerance &&
           yaw_distance <= global_localization_consistency_yaw_tolerance;
  }

  int update_global_localization_consistency(const Eigen::Isometry3f& pose) {
    PendingGlobalLocalizationCandidate candidate;
    candidate.pose = pose;
    pending_global_localization_candidates.push_back(candidate);

    const size_t max_candidates = static_cast<size_t>(std::max(1, global_localization_consistency_window));
    while (pending_global_localization_candidates.size() > max_candidates) {
      pending_global_localization_candidates.erase(pending_global_localization_candidates.begin());
    }

    int consistent_count = 0;
    for (const auto& existing : pending_global_localization_candidates) {
      if (poses_consistent(existing.pose, pose)) {
        ++consistent_count;
      }
    }
    return consistent_count;
  }

  bool handle_global_pose_probation(bool correction_accepted) {
    if (global_pose_probation_frames_remaining <= 0) {
      return true;
    }

    --global_pose_probation_frames_remaining;
    if (!correction_accepted) {
      ++global_pose_probation_rejections;
    }

    if (global_pose_probation_rejections >= std::max(1, global_localization_post_accept_max_rejections)) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "revoke global localization result during post-accept validation: rejections="
          << global_pose_probation_rejections
          << "/" << std::max(1, global_localization_post_accept_max_rejections));
      pose_estimator.reset();
      auto_relocalize_done = false;
      relocalize_requested = true;
      global_pose_probation_frames_remaining = 0;
      global_pose_probation_rejections = 0;
      consecutive_scan_matching_rejections = 0;
      return false;
    }

    if (global_pose_probation_frames_remaining == 0) {
      RCLCPP_INFO(get_logger(), "global localization result passed post-accept scan matching validation");
    }
    return true;
  }

  void apply_2d_constraints(Eigen::Isometry3f& pose) const {
    if (!force_2d_pose) {
      return;
    }

    const float yaw = std::atan2(pose.matrix()(1, 0), pose.matrix()(0, 0));
    pose.linear() = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    if (force_2d_fixed_z) {
      pose.translation().z() = global_localization_pose_z_offset;
    }
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg) {
    RCLCPP_INFO(get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    auto_relocalize_done = true;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = pose_msg->header;
    pose.pose = pose_msg->pose.pose;

    if (!pose.header.frame_id.empty() && pose.header.frame_id != "map") {
      try {
        geometry_msgs::msg::TransformStamped map_transform =
          tf_buffer->lookupTransform("map", pose.header.frame_id, zero_time(), rclcpp::Duration(std::chrono::milliseconds(100)));
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, map_transform);
        pose = transformed_pose;
        RCLCPP_INFO_STREAM(get_logger(), "transformed initial pose from " << pose_msg->header.frame_id << " to map");
      } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_STREAM(get_logger(), "failed to transform initial pose from " << pose_msg->header.frame_id << " to map: " << e.what());
        return;
      }
    }

    const auto& p = pose.pose.position;
    const auto& q = pose.pose.orientation;
    pose_estimator.reset(
      new hdl_localization::PoseEstimator(
        registration,
        get_clock()->now(),
        Eigen::Vector3f(p.x, p.y, p.z),
        Eigen::Quaternionf(q.w, q.x, q.y, q.z),
        cool_time_duration,
        force_2d_pose,
        force_2d_fixed_z ? global_localization_pose_z_offset : std::numeric_limits<double>::quiet_NaN()));
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    if (send_tf_transforms) {
      if (tf_buffer->canTransform(robot_odom_frame_id, odom_child_frame_id, zero_time())) {
        geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
        map_wrt_frame.header.stamp = stamp;
        map_wrt_frame.header.frame_id = odom_child_frame_id;
        map_wrt_frame.child_frame_id = "map";

        geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer->lookupTransform(
          robot_odom_frame_id,
          odom_child_frame_id,
          zero_time(),
          rclcpp::Duration(std::chrono::milliseconds(100)));
        Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

        geometry_msgs::msg::TransformStamped map_wrt_odom;
        tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

        tf2::Transform odom_wrt_map;
        tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
        odom_wrt_map = odom_wrt_map.inverse();

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.transform = tf2::toMsg(odom_wrt_map);
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = robot_odom_frame_id;

        tf_broadcaster->sendTransform(odom_trans);
      } else {
        geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = odom_child_frame_id;
        tf_broadcaster->sendTransform(odom_trans);
      }
    }

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    // odom.pose.pose.position.x = pose_trans.transform.translation.x;
    // odom.pose.pose.position.y = pose_trans.transform.translation.y;
    // odom.pose.pose.position.z = pose_trans.transform.translation.z;
    // odom.pose.pose.orientation = pose_trans.transform.rotation;
    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub->publish(odom);
  }

  void publish_scan_matching_status(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    msg::ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration->hasConverged();
    status.matching_error = registration->getFitnessScore();

    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    if (aligned->empty()) {
      status.inlier_fraction = 0.0f;
      status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;
      status_pub->publish(status);
      return;
    }

    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for (int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    std::vector<double> errors(6, 0.0);

    if (pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }

    if (pose_estimator->imu_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = use_imu ? "imu" : "motion_model";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
    }

    if (pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }

    status_pub->publish(status);
  }

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  bool send_tf_transforms;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
  rclcpp::Publisher<hdl_localization::msg::ScanMatchingStatus>::SharedPtr status_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // global localization
  bool use_global_localization;
  bool auto_relocalize_on_start;
  int auto_relocalize_after_rejections;
  double global_localization_pose_z_offset;
  bool validate_global_localization_with_scan_matching;
  double global_localization_max_fitness_score;
  int global_localization_max_candidates;
  double global_localization_min_fitness_margin;
  int global_localization_required_consistent_results;
  int global_localization_consistency_window;
  double global_localization_consistency_xy_tolerance;
  double global_localization_consistency_yaw_tolerance;
  int global_localization_query_accumulation_frames;
  int global_localization_query_min_accumulation_frames;
  int global_localization_post_accept_validation_frames;
  int global_localization_post_accept_max_rejections;
  bool force_2d_pose;
  bool force_2d_fixed_z;
  bool global_localization_use_height_filter;
  double global_localization_min_z;
  double global_localization_max_z;
  bool global_localization_use_max_z_filter;
  double global_localization_query_timeout_sec;
  std::atomic_bool relocalizing;
  std::atomic_bool globalmap_set_for_global_localization;
  std::atomic_bool auto_relocalize_done;
  std::atomic_bool relocalize_requested;
  std::unique_ptr<DeltaEstimater> delta_estimater;
  int consecutive_scan_matching_rejections;
  int global_pose_probation_frames_remaining;
  int global_pose_probation_rejections;
  std::vector<PendingGlobalLocalizationCandidate> pending_global_localization_candidates;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  std::deque<pcl::PointCloud<PointT>::ConstPtr> recent_scans;
  rclcpp::CallbackGroup::SharedPtr global_localization_callback_group;
  rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_global_map_service;
  rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_server;
  rclcpp::TimerBase::SharedPtr auto_relocalize_timer;

  // Parameters
  double cool_time_duration;
  std::string reg_method;
  std::string ndt_neighbor_search_method;
  double ndt_neighbor_search_radius;
  double ndt_resolution;
  bool enable_robot_odometry_prediction;
  bool reject_scan_matching_without_convergence;
  double max_scan_matching_fitness_score;
};
}  // namespace hdl_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalizationNodelet)
