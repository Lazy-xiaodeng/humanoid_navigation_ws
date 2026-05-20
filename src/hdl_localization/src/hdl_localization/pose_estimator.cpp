#include <hdl_localization/pose_estimator.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <hdl_localization/pose_system.hpp>
#include <hdl_localization/odom_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

namespace {

float pose_yaw(const Eigen::Matrix4f& pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

float normalize_angle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Matrix4f project_pose_to_2d(const Eigen::Matrix4f& pose, double fixed_z) {
  Eigen::Matrix4f projected = Eigen::Matrix4f::Identity();
  const float yaw = pose_yaw(pose);
  projected.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  projected.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3);
  if (std::isfinite(fixed_z)) {
    projected(2, 3) = fixed_z;
  }
  return projected;
}

Eigen::Matrix4f project_delta_to_2d(const Eigen::Matrix4f& delta) {
  Eigen::Matrix4f projected = Eigen::Matrix4f::Identity();
  const float yaw = pose_yaw(delta);
  projected.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  projected(0, 3) = delta(0, 3);
  projected(1, 3) = delta(1, 3);
  projected(2, 3) = 0.0f;
  return projected;
}

void constrain_state_to_2d(
  Eigen::VectorXf& mean,
  int quat_offset,
  double fixed_z,
  int velocity_z_offset = -1) {
  Eigen::Quaternionf quat(
    mean[quat_offset],
    mean[quat_offset + 1],
    mean[quat_offset + 2],
    mean[quat_offset + 3]);
  quat.normalize();

  const Eigen::Matrix3f rot = quat.toRotationMatrix();
  const float yaw = std::atan2(rot(1, 0), rot(0, 0));
  const Eigen::Quaternionf yaw_quat(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

  if (std::isfinite(fixed_z)) {
    mean[2] = static_cast<float>(fixed_z);
  }
  if (velocity_z_offset >= 0) {
    mean[velocity_z_offset] = 0.0f;
  }
  mean[quat_offset] = yaw_quat.w();
  mean[quat_offset + 1] = yaw_quat.x();
  mean[quat_offset + 2] = yaw_quat.y();
  mean[quat_offset + 3] = yaw_quat.z();
}

double compute_inlier_fraction(
  const pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr& registration,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& aligned,
  double max_correspondence_dist) {
  if (!aligned || aligned->empty()) {
    return 0.0;
  }

  auto target_search = registration->getSearchMethodTarget();
  if (!target_search || max_correspondence_dist <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double max_sq_dist = max_correspondence_dist * max_correspondence_dist;
  int num_inliers = 0;
  int num_valid_points = 0;
  std::vector<int> k_indices;
  std::vector<float> k_sq_dists;
  k_indices.reserve(1);
  k_sq_dists.reserve(1);

  for (const auto& pt : aligned->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    ++num_valid_points;
    k_indices.clear();
    k_sq_dists.clear();
    if (target_search->nearestKSearch(pt, 1, k_indices, k_sq_dists) > 0 && !k_sq_dists.empty() && k_sq_dists[0] < max_sq_dist) {
      ++num_inliers;
    }
  }

  if (num_valid_points == 0) {
    return 0.0;
  }
  return static_cast<double>(num_inliers) / static_cast<double>(num_valid_points);
}

}  // namespace

/**
 * @brief constructor
 * @param registration        registration method
 * @param stamp               timestamp
 * @param pos                 initial position
 * @param quat                initial orientation
 * @param cool_time_duration  during "cool time", prediction is not performed
 */
PoseEstimator::PoseEstimator(
  pcl::Registration<PointT, PointT>::Ptr& registration,
  const rclcpp::Time& stamp,
  const Eigen::Vector3f& pos,
  const Eigen::Quaternionf& quat,
  double cool_time_duration,
  bool force_2d_pose,
  double fixed_z)
    : init_stamp(stamp),
      registration(registration),
      cool_time_duration(cool_time_duration),
      force_2d_pose(force_2d_pose),
      fixed_z(fixed_z) {

  prev_stamp = rclcpp::Time((int64_t)0, init_stamp.get_clock_type());
  last_correction_stamp = rclcpp::Time((int64_t)0, init_stamp.get_clock_type());
  last_observation = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf init_quat = quat;
  Eigen::Vector3f init_pos = pos;
  if (force_2d_pose) {
    const Eigen::Matrix3f init_rot = quat.toRotationMatrix();
    const float yaw = std::atan2(init_rot(1, 0), init_rot(0, 0));
    init_quat = Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    if (std::isfinite(fixed_z)) {
      init_pos.z() = fixed_z;
    }
  }
  last_observation.block<3, 3>(0, 0) = init_quat.toRotationMatrix();
  last_observation.block<3, 1>(0, 3) = init_pos;

  process_noise = Eigen::MatrixXf::Identity(16, 16);
  process_noise.middleRows(0, 3) *= 1.0;
  process_noise.middleRows(3, 3) *= 1.0;
  process_noise.middleRows(6, 4) *= 0.5;
  process_noise.middleRows(10, 3) *= 1e-6;
  process_noise.middleRows(13, 3) *= 1e-6;

  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= 0.01;
  measurement_noise.middleRows(3, 4) *= 0.001;

  Eigen::VectorXf mean(16);
  mean.middleRows(0, 3) = init_pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(init_quat.w(), init_quat.x(), init_quat.y(), init_quat.z());
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3).setZero();

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

  PoseSystem system;
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator() {}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const rclcpp::Time& stamp) {
  if ((stamp - init_stamp).seconds() < cool_time_duration || prev_stamp == rclcpp::Time((int64_t)0, prev_stamp.get_clock_type()) || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).seconds();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  ukf->predict();
  if (force_2d_pose) {
    constrain_state_to_2d(ukf->mean, 6, fixed_z, 5);
  }
}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const rclcpp::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
  if ((stamp - init_stamp).seconds() < cool_time_duration || prev_stamp == rclcpp::Time((int64_t)0, prev_stamp.get_clock_type()) || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).seconds();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;

  ukf->predict(control);
  if (force_2d_pose) {
    constrain_state_to_2d(ukf->mean, 6, fixed_z, 5);
  }
}

/**
 * @brief update the state of the odomety-based pose estimation
 */
void PoseEstimator::predict_odom(const Eigen::Matrix4f& odom_delta) {
  if(!odom_ukf) {
    Eigen::MatrixXf odom_process_noise = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf odom_measurement_noise = Eigen::MatrixXf::Identity(7, 7) * 1e-3;

    Eigen::Matrix4f initial_pose = matrix();
    if (force_2d_pose) {
      initial_pose = project_pose_to_2d(initial_pose, fixed_z);
    }

    Eigen::VectorXf odom_mean(7);
    odom_mean.block<3, 1>(0, 0) = initial_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf initial_quat(initial_pose.block<3, 3>(0, 0));
    odom_mean.block<4, 1>(3, 0) =
      Eigen::Vector4f(initial_quat.w(), initial_quat.x(), initial_quat.y(), initial_quat.z());
    Eigen::MatrixXf odom_cov = Eigen::MatrixXf::Identity(7, 7) * 1e-2;

    OdomSystem odom_system;
    odom_ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>(odom_system, 7, 7, 7, odom_process_noise, odom_measurement_noise, odom_mean, odom_cov));
  }

  Eigen::Matrix4f constrained_delta = odom_delta;
  if (force_2d_pose) {
    // The Fast-LIO TF chain is 6D, but Nav2 localization publishes a planar
    // map->odom correction. Keep odom prediction in the same x/y/yaw manifold.
    constrained_delta = project_delta_to_2d(odom_delta);
  }

  // invert quaternion if the rotation axis is flipped
  Eigen::Quaternionf quat(constrained_delta.block<3, 3>(0, 0));
  if(odom_quat().coeffs().dot(quat.coeffs()) < 0.0) {
    quat.coeffs() *= -1.0f;
  }

  Eigen::VectorXf control(7);
  control.middleRows(0, 3) = constrained_delta.block<3, 1>(0, 3);
  control.middleRows(3, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());

  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(7, 7);
  process_noise.topLeftCorner(3, 3) =
    Eigen::Matrix3f::Identity() * constrained_delta.block<3, 1>(0, 3).norm() +
    Eigen::Matrix3f::Identity() * 1e-3;
  process_noise.bottomRightCorner(4, 4) = Eigen::Matrix4f::Identity() * (1 - std::abs(quat.w())) + Eigen::Matrix4f::Identity() * 1e-3;

  odom_ukf->setProcessNoiseCov(process_noise);
  odom_ukf->predict(control);

  if (force_2d_pose) {
    constrain_state_to_2d(odom_ukf->mean, 3, fixed_z);
  }
}

/**
 * @brief correct
 * @param cloud   input cloud
 * @return cloud aligned to the globalmap
 */
pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct(
  const rclcpp::Time& stamp,
  const pcl::PointCloud<PointT>::ConstPtr& cloud,
  bool reject_non_converged,
  double max_fitness_score,
  bool* correction_accepted,
  double max_correction_translation,
  double max_correction_yaw,
  bool* correction_rejected_by_jump,
  double min_inlier_fraction,
  double max_inlier_distance,
  double* inlier_fraction,
  bool* correction_rejected_by_inlier,
  double jump_override_max_fitness_score,
  double jump_override_min_inlier_fraction,
  bool* correction_jump_overridden) {
  if (correction_accepted) {
    *correction_accepted = false;
  }
  if (correction_rejected_by_jump) {
    *correction_rejected_by_jump = false;
  }
  if (inlier_fraction) {
    *inlier_fraction = std::numeric_limits<double>::quiet_NaN();
  }
  if (correction_rejected_by_inlier) {
    *correction_rejected_by_inlier = false;
  }
  if (correction_jump_overridden) {
    *correction_jump_overridden = false;
  }

  Eigen::Matrix4f no_guess = last_observation;
  Eigen::Matrix4f imu_guess;
  Eigen::Matrix4f odom_guess;
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

  if(!odom_ukf) {
    init_guess = imu_guess = matrix();
  } else {
    imu_guess = matrix();
    odom_guess = odom_matrix();

    Eigen::VectorXf imu_mean(7);
    Eigen::MatrixXf imu_cov = Eigen::MatrixXf::Identity(7, 7);
    imu_mean.block<3, 1>(0, 0) = ukf->mean.block<3, 1>(0, 0);
    imu_mean.block<4, 1>(3, 0) = ukf->mean.block<4, 1>(6, 0);

    imu_cov.block<3, 3>(0, 0) = ukf->cov.block<3, 3>(0, 0);
    imu_cov.block<3, 4>(0, 3) = ukf->cov.block<3, 4>(0, 6);
    imu_cov.block<4, 3>(3, 0) = ukf->cov.block<4, 3>(6, 0);
    imu_cov.block<4, 4>(3, 3) = ukf->cov.block<4, 4>(6, 6);

    Eigen::VectorXf odom_mean = odom_ukf->mean;
    Eigen::MatrixXf odom_cov = odom_ukf->cov;

    if (imu_mean.tail<4>().dot(odom_mean.tail<4>()) < 0.0) {
      odom_mean.tail<4>() *= -1.0;
    }

    Eigen::MatrixXf inv_imu_cov = imu_cov.inverse();
    Eigen::MatrixXf inv_odom_cov = odom_cov.inverse();

    Eigen::MatrixXf fused_cov = (inv_imu_cov + inv_odom_cov).inverse();
    Eigen::VectorXf fused_mean = fused_cov * inv_imu_cov * imu_mean + fused_cov * inv_odom_cov * odom_mean;

    init_guess.block<3, 1>(0, 3) = Eigen::Vector3f(fused_mean[0], fused_mean[1], fused_mean[2]);
    init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(fused_mean[3], fused_mean[4], fused_mean[5], fused_mean[6]).normalized().toRotationMatrix();
    if (force_2d_pose) {
      init_guess = project_pose_to_2d(init_guess, fixed_z);
    }
  }

  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  registration->setInputSource(cloud);
  registration->align(*aligned, init_guess);
  last_correction_stamp = stamp;

  const bool converged = registration->hasConverged();
  const double fitness_score = registration->getFitnessScore();
  const bool fitness_score_valid = std::isfinite(fitness_score);
  if ((reject_non_converged && !converged) || (max_fitness_score > 0.0 && (!fitness_score_valid || fitness_score > max_fitness_score))) {
    return aligned;
  }

  double current_inlier_fraction = std::numeric_limits<double>::quiet_NaN();
  if (min_inlier_fraction > 0.0 || inlier_fraction || jump_override_min_inlier_fraction > 0.0) {
    current_inlier_fraction = compute_inlier_fraction(registration, aligned, max_inlier_distance);
    if (inlier_fraction) {
      *inlier_fraction = current_inlier_fraction;
    }
    if (min_inlier_fraction > 0.0 && (!std::isfinite(current_inlier_fraction) || current_inlier_fraction < min_inlier_fraction)) {
      if (correction_rejected_by_inlier) {
        *correction_rejected_by_inlier = true;
      }
      return aligned;
    }
  }

  Eigen::Matrix4f trans = registration->getFinalTransformation();
  if (force_2d_pose) {
    const float yaw = std::atan2(trans(1, 0), trans(0, 0));
    trans.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    if (std::isfinite(fixed_z)) {
      trans(2, 3) = fixed_z;
    }
  }

  const bool within_cool_time = (stamp - init_stamp).seconds() < cool_time_duration;
  if (!within_cool_time) {
    const Eigen::Matrix4f prediction = odom_ukf ? odom_guess : init_guess;
    const float correction_translation =
      (trans.block<2, 1>(0, 3) - prediction.block<2, 1>(0, 3)).norm();
    const float correction_yaw = std::abs(normalize_angle(pose_yaw(trans) - pose_yaw(prediction)));
    if (
      (max_correction_translation > 0.0 && correction_translation > max_correction_translation) ||
      (max_correction_yaw > 0.0 && correction_yaw > max_correction_yaw)) {
      const bool fitness_confident =
        jump_override_max_fitness_score <= 0.0 ||
        (fitness_score_valid && fitness_score <= jump_override_max_fitness_score);
      const bool inlier_confident =
        jump_override_min_inlier_fraction <= 0.0 ||
        (std::isfinite(current_inlier_fraction) && current_inlier_fraction >= jump_override_min_inlier_fraction);
      const bool jump_override_enabled =
        jump_override_max_fitness_score > 0.0 || jump_override_min_inlier_fraction > 0.0;
      if (jump_override_enabled && fitness_confident && inlier_confident) {
        if (correction_jump_overridden) {
          *correction_jump_overridden = true;
        }
      } else {
        if (correction_rejected_by_jump) {
          *correction_rejected_by_jump = true;
        }
        return aligned;
      }
    }
  }

  Eigen::Vector3f p = trans.block<3, 1>(0, 3);
  Eigen::Quaternionf q(trans.block<3, 3>(0, 0));

  if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
    q.coeffs() *= -1.0f;
  }

  Eigen::VectorXf observation(7);
  observation.middleRows(0, 3) = p;
  observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
  last_observation = trans;

  wo_pred_error = no_guess.inverse() * registration->getFinalTransformation();

  ukf->correct(observation);
  if (force_2d_pose) {
    constrain_state_to_2d(ukf->mean, 6, fixed_z, 5);
  }
  imu_pred_error = imu_guess.inverse() * registration->getFinalTransformation();

  if(odom_ukf) {
    if (observation.tail<4>().dot(odom_ukf->mean.tail<4>()) < 0.0) {
      odom_ukf->mean.tail<4>() *= -1.0;
    }

    odom_ukf->correct(observation);
    if (force_2d_pose) {
      constrain_state_to_2d(odom_ukf->mean, 3, fixed_z);
    }
    odom_pred_error = odom_guess.inverse() * registration->getFinalTransformation();
  }

  if (correction_accepted) {
    *correction_accepted = true;
  }

  return aligned;
}

/* getters */
rclcpp::Time PoseEstimator::last_correction_time() const {
  return last_correction_stamp;
}

Eigen::Vector3f PoseEstimator::pos() const {
  return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
}

Eigen::Vector3f PoseEstimator::vel() const {
  return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
}

Eigen::Quaternionf PoseEstimator::quat() const {
  return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
}

Eigen::Matrix4f PoseEstimator::matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = pos();
  if (force_2d_pose) {
    m = project_pose_to_2d(m, fixed_z);
  }
  return m;
}

Eigen::Vector3f PoseEstimator::odom_pos() const {
  return Eigen::Vector3f(odom_ukf->mean[0], odom_ukf->mean[1], odom_ukf->mean[2]);
}

Eigen::Quaternionf PoseEstimator::odom_quat() const {
  return Eigen::Quaternionf(odom_ukf->mean[3], odom_ukf->mean[4], odom_ukf->mean[5], odom_ukf->mean[6]).normalized();
}

Eigen::Matrix4f PoseEstimator::odom_matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = odom_quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = odom_pos();
  if (force_2d_pose) {
    m = project_pose_to_2d(m, fixed_z);
  }
  return m;
}

bool PoseEstimator::has_odom_prediction() const {
  return static_cast<bool>(odom_ukf);
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error() const {
  return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error() const {
  return imu_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::odom_prediction_error() const {
  return odom_pred_error;
}
}
