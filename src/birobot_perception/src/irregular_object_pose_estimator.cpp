#include "birobot_perception/irregular_object_pose_estimator.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace birobot_perception
{

IrregularObjectPoseEstimator::IrregularObjectPoseEstimator(const rclcpp::NodeOptions & options)
: LifecycleNode("birobot_perception_node", options),
  last_processing_latency_ms_(0.0),
  last_inlier_pct_(0.0),
  tracked_objects_count_(0),
  last_cloud_stamp_(0, 0, get_clock()->get_clock_type())
{
}

IrregularObjectPoseEstimator::CallbackReturn
IrregularObjectPoseEstimator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring birobot_perception_node...");

  // Declare parameters
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/birobot/depth_camera/points");
  target_frame_ = declare_parameter<std::string>("target_frame", "world");
  target_tf_prefix_ = declare_parameter<std::string>("target_tf_prefix", "grasp_target_");
  ransac_distance_threshold_ = declare_parameter<double>("ransac_distance_threshold", 0.015);
  ransac_max_iterations_ = declare_parameter<int>("ransac_max_iterations", 1000);
  cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.02);
  min_cluster_size_ = declare_parameter<int>("min_cluster_size", 50);
  max_cluster_size_ = declare_parameter<int>("max_cluster_size", 25000);
  max_tracked_objects_ = declare_parameter<int>("max_tracked_objects", 2);
  diagnostic_rate_hz_ = declare_parameter<double>("diagnostic_rate_hz", 1.0);
  min_valid_points_ = declare_parameter<int>("min_valid_points", 1000);

  // Initialize publishers
  pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  pub_object_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/birobot/perception/object_cloud", 5);
  pub_target_poses_ = create_publisher<geometry_msgs::msg::PoseArray>("/birobot/perception/target_poses", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Configuration successful.");
  return CallbackReturn::SUCCESS;
}

IrregularObjectPoseEstimator::CallbackReturn
IrregularObjectPoseEstimator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating birobot_perception_node...");

  pub_diagnostics_->on_activate();
  pub_object_cloud_->on_activate();
  pub_target_poses_->on_activate();

  sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&IrregularObjectPoseEstimator::pointcloud_callback, this, std::placeholders::_1));

  auto diag_period = std::chrono::duration<double>(1.0 / diagnostic_rate_hz_);
  timer_diagnostics_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(diag_period),
    std::bind(&IrregularObjectPoseEstimator::publish_diagnostics, this));

  RCLCPP_INFO(get_logger(), "Activation successful. Subscribed to %s", input_cloud_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

IrregularObjectPoseEstimator::CallbackReturn
IrregularObjectPoseEstimator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating birobot_perception_node...");

  sub_pointcloud_.reset();
  timer_diagnostics_.reset();

  pub_diagnostics_->on_deactivate();
  pub_object_cloud_->on_deactivate();
  pub_target_poses_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivation successful.");
  return CallbackReturn::SUCCESS;
}

IrregularObjectPoseEstimator::CallbackReturn
IrregularObjectPoseEstimator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up birobot_perception_node...");

  pub_diagnostics_.reset();
  pub_object_cloud_.reset();
  pub_target_poses_.reset();
  tf_broadcaster_.reset();

  return CallbackReturn::SUCCESS;
}

IrregularObjectPoseEstimator::CallbackReturn
IrregularObjectPoseEstimator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down birobot_perception_node...");
  return CallbackReturn::SUCCESS;
}

bool IrregularObjectPoseEstimator::segment_table_plane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_cloud,
  double & inlier_pct)
{
  if (!input_cloud || input_cloud->empty()) {
    inlier_pct = 0.0;
    return false;
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations_);
  seg.setDistanceThreshold(ransac_distance_threshold_);

  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    inlier_pct = 0.0;
    *non_table_cloud = *input_cloud;
    return false;
  }

  inlier_pct = (static_cast<double>(inliers->indices.size()) / input_cloud->size()) * 100.0;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*non_table_cloud);

  return true;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
IrregularObjectPoseEstimator::extract_clusters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_cloud)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  if (!non_table_cloud || non_table_cloud->size() < static_cast<size_t>(min_cluster_size_)) {
    return clusters;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(non_table_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(non_table_cloud);
  ec.extract(cluster_indices);

  for (const auto & indices : cluster_indices) {
    if (static_cast<int>(clusters.size()) >= max_tracked_objects_) {
      break;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & idx : indices.indices) {
      cluster->points.push_back((*non_table_cloud)[idx]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
  }

  return clusters;
}

bool IrregularObjectPoseEstimator::compute_cluster_pose(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
  geometry_msgs::msg::Pose & pose)
{
  if (!cluster || cluster->size() < 3) {
    return false;
  }

  // 1. Centroid calculation
  Eigen::Vector4f centroid_4f;
  pcl::compute3DCentroid(*cluster, centroid_4f);

  // 2. Normalized Covariance Matrix calculation
  Eigen::Matrix3f covariance_matrix;
  pcl::computeCovarianceMatrixNormalized(*cluster, centroid_4f, covariance_matrix);

  // 3. Eigen decomposition (PCA)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
  if (eigensolver.info() != Eigen::Success) {
    return false;
  }

  // Eigenvectors sorted by ascending eigenvalue. Col(2) is primary axis.
  Eigen::Vector3f primary_axis = eigensolver.eigenvectors().col(2);

  // Project primary axis onto XY plane for yaw orientation
  Eigen::Vector3f x_gripper = primary_axis;
  x_gripper.z() = 0.0f;
  if (x_gripper.norm() < 1e-4f) {
    x_gripper = Eigen::Vector3f::UnitX();
  } else {
    x_gripper.normalize();
  }

  // Approach Z constrained downward
  Eigen::Vector3f z_gripper(0.0f, 0.0f, -1.0f);
  Eigen::Vector3f y_gripper = z_gripper.cross(x_gripper).normalized();
  x_gripper = y_gripper.cross(z_gripper).normalized();

  Eigen::Matrix3f rot_matrix;
  rot_matrix.col(0) = x_gripper;
  rot_matrix.col(1) = y_gripper;
  rot_matrix.col(2) = z_gripper;

  Eigen::Quaternionf q(rot_matrix);
  q.normalize();

  pose.position.x = centroid_4f[0];
  pose.position.y = centroid_4f[1];
  pose.position.z = centroid_4f[2];
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return true;
}

void IrregularObjectPoseEstimator::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  last_cloud_stamp_ = msg->header.stamp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *input_cloud);

  if (static_cast<int>(input_cloud->size()) < min_valid_points_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received point cloud with only %zu points (minimum threshold: %d)",
      input_cloud->size(), min_valid_points_);
    tracked_objects_count_ = 0;
    last_processing_latency_ms_ = 0.0;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  segment_table_plane(input_cloud, non_table_cloud, last_inlier_pct_);

  auto clusters = extract_clusters(non_table_cloud);
  tracked_objects_count_ = static_cast<int>(clusters.size());

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = msg->header.stamp;
  pose_array.header.frame_id = target_frame_;

  pcl::PointCloud<pcl::PointXYZ> combined_object_cloud;

  for (size_t i = 0; i < clusters.size(); ++i) {
    geometry_msgs::msg::Pose target_pose;
    if (compute_cluster_pose(clusters[i], target_pose)) {
      pose_array.poses.push_back(target_pose);

      // Broadcast dynamic TF
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = msg->header.stamp;
      tf_msg.header.frame_id = target_frame_;
      tf_msg.child_frame_id = target_tf_prefix_ + std::to_string(i + 1);

      tf_msg.transform.translation.x = target_pose.position.x;
      tf_msg.transform.translation.y = target_pose.position.y;
      tf_msg.transform.translation.z = target_pose.position.z;
      tf_msg.transform.rotation = target_pose.orientation;

      tf_broadcaster_->sendTransform(tf_msg);
    }
    combined_object_cloud += *clusters[i];
  }

  if (pub_target_poses_ && pub_target_poses_->is_activated()) {
    pub_target_poses_->publish(pose_array);
  }

  if (pub_object_cloud_ && pub_object_cloud_->is_activated()) {
    sensor_msgs::msg::PointCloud2 object_cloud_msg;
    pcl::toROSMsg(combined_object_cloud, object_cloud_msg);
    object_cloud_msg.header.stamp = msg->header.stamp;
    object_cloud_msg.header.frame_id = msg->header.frame_id;
    pub_object_cloud_->publish(object_cloud_msg);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  last_processing_latency_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

void IrregularObjectPoseEstimator::publish_diagnostics()
{
  if (!pub_diagnostics_ || !pub_diagnostics_->is_activated()) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "birobot_perception: irregular_object_pose_estimator";
  status.hardware_id = "gazebo_depth_camera";

  if (tracked_objects_count_ > 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Tracking " + std::to_string(tracked_objects_count_) + " target objects";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "No target objects detected on table plane";
  }

  diagnostic_msgs::msg::KeyValue kv_latency;
  kv_latency.key = "processing_latency_ms";
  kv_latency.value = std::to_string(last_processing_latency_ms_);
  status.values.push_back(kv_latency);

  diagnostic_msgs::msg::KeyValue kv_tracked;
  kv_tracked.key = "tracked_targets_count";
  kv_tracked.value = std::to_string(tracked_objects_count_);
  status.values.push_back(kv_tracked);

  diagnostic_msgs::msg::KeyValue kv_inliers;
  kv_inliers.key = "ransac_plane_inliers_pct";
  kv_inliers.value = std::to_string(last_inlier_pct_);
  status.values.push_back(kv_inliers);

  diag_msg.status.push_back(status);
  pub_diagnostics_->publish(diag_msg);
}

}  // namespace birobot_perception
