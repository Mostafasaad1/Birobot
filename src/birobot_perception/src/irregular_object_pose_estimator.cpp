#include "birobot_perception/irregular_object_pose_estimator.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/birobot/depth_camera/points/points");
  input_rgb_image_topic_ = declare_parameter<std::string>("input_rgb_image_topic", "/birobot/depth_camera/image");
  target_frame_ = declare_parameter<std::string>("target_frame", "world");
  target_tf_prefix_ = declare_parameter<std::string>("target_tf_prefix", "grasp_target_");
  ransac_distance_threshold_ = declare_parameter<double>("ransac_distance_threshold", 0.015);
  ransac_max_iterations_ = declare_parameter<int>("ransac_max_iterations", 1000);
  cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.02);
  min_cluster_size_ = declare_parameter<int>("min_cluster_size", 50);
  max_cluster_size_ = declare_parameter<int>("max_cluster_size", 25000);
  max_tracked_objects_ = declare_parameter<int>("max_tracked_objects", 2);
  diagnostic_rate_hz_ = declare_parameter<double>("diagnostic_rate_hz", 1.0);
  min_valid_points_ = declare_parameter<int>("min_valid_points", 100);

  // Workspace Bounding Box Parameters (crops out overhead robot arms, floor, outside tables)
  workspace_min_x_ = declare_parameter<double>("workspace_min_x", -1.25);
  workspace_max_x_ = declare_parameter<double>("workspace_max_x", 0.45);
  workspace_min_y_ = declare_parameter<double>("workspace_min_y", -0.60);
  workspace_max_y_ = declare_parameter<double>("workspace_max_y", 0.60);
  workspace_min_z_ = declare_parameter<double>("workspace_min_z", -0.05);
  workspace_max_z_ = declare_parameter<double>("workspace_max_z", 0.35);

  // RGB-D Color Parameters
  enable_color_filtering_ = declare_parameter<bool>("enable_color_filtering", true);
  hsv_red_h_low_1_ = declare_parameter<int>("hsv_red_h_low_1", 0);
  hsv_red_h_high_1_ = declare_parameter<int>("hsv_red_h_high_1", 15);
  hsv_red_h_low_2_ = declare_parameter<int>("hsv_red_h_low_2", 165);
  hsv_red_h_high_2_ = declare_parameter<int>("hsv_red_h_high_2", 180);
  hsv_red_s_min_ = declare_parameter<int>("hsv_red_s_min", 70);
  hsv_red_v_min_ = declare_parameter<int>("hsv_red_v_min", 40);

  // Initialize publishers
  pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  pub_object_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/birobot/perception/object_cloud", 5);
  pub_target_poses_ = create_publisher<geometry_msgs::msg::PoseArray>("/birobot/perception/target_poses", 10);
  pub_red_object_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/birobot/perception/red_object_pose", 10);
  pub_annotated_image_ = create_publisher<sensor_msgs::msg::Image>("/birobot/perception/red_object_detection_image", 5);
  pub_target_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/birobot/perception/target_markers", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
  pub_red_object_pose_->on_activate();
  pub_annotated_image_->on_activate();
  pub_target_markers_->on_activate();

  sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&IrregularObjectPoseEstimator::pointcloud_callback, this, std::placeholders::_1));

  sub_rgb_image_ = create_subscription<sensor_msgs::msg::Image>(
    input_rgb_image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&IrregularObjectPoseEstimator::rgb_image_callback, this, std::placeholders::_1));

  auto diag_period = std::chrono::duration<double>(1.0 / diagnostic_rate_hz_);
  timer_diagnostics_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(diag_period),
    std::bind(&IrregularObjectPoseEstimator::publish_diagnostics, this));

  RCLCPP_INFO(get_logger(), "Activation successful. Subscribed to %s and %s",
    input_cloud_topic_.c_str(), input_rgb_image_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

IrregularObjectPoseEstimator::CallbackReturn
IrregularObjectPoseEstimator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating birobot_perception_node...");

  sub_pointcloud_.reset();
  sub_rgb_image_.reset();
  timer_diagnostics_.reset();

  pub_diagnostics_->on_deactivate();
  pub_object_cloud_->on_deactivate();
  pub_target_poses_->on_deactivate();
  pub_red_object_pose_->on_deactivate();
  pub_annotated_image_->on_deactivate();
  pub_target_markers_->on_deactivate();

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
  pub_red_object_pose_.reset();
  pub_annotated_image_.reset();
  pub_target_markers_.reset();
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
  size_t total_inliers = 0;

  // Segment and remove up to 2 dominant horizontal supporting planes (table at z~0.05 and ground at z~0.00)
  for (int iter = 0; iter < 2; ++iter) {
    if (current_cloud->size() < 100) {
      break;
    }

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setDistanceThreshold(ransac_distance_threshold_);

    seg.setInputCloud(current_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      break;
    }

    // A valid supporting plane must have at least 15% of current cloud or >= 300 points
    size_t min_plane_pts = std::min(static_cast<size_t>(300), static_cast<size_t>(current_cloud->size() * 0.15));
    if (inliers->indices.size() < min_plane_pts) {
      break;
    }

    // Must be roughly horizontal (|nz| >= 0.80)
    if (coefficients->values.size() >= 3) {
      double nz = std::abs(coefficients->values[2]);
      if (nz < 0.80) {
        break;
      }
    }

    total_inliers += inliers->indices.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(current_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*remaining);
    current_cloud = remaining;
  }

  if (total_inliers == 0) {
    inlier_pct = 0.0;
    *non_table_cloud = *input_cloud;
    return false;
  }

  *non_table_cloud = *current_cloud;
  inlier_pct = (static_cast<double>(total_inliers) / input_cloud->size()) * 100.0;
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

  // 3. 2D PCA in the XY horizontal table plane.
  // The table is horizontal (Z normal) and gripper approaches top-down along -Z.
  // Grasping requires determining the object length orientation in the XY plane.
  // Using 2D PCA in XY avoids contamination from the vertical height Z
  // (e.g. when object height 0.20m > length 0.15m).
  Eigen::Matrix2f cov_2d = covariance_matrix.block<2, 2>(0, 0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver_2d(cov_2d);
  if (eigensolver_2d.info() != Eigen::Success) {
    return false;
  }

  // Col(1) is the eigenvector with the largest eigenvalue in the XY plane (object length)
  Eigen::Vector2f primary_axis_2d = eigensolver_2d.eigenvectors().col(1);
  if (primary_axis_2d.norm() < 1e-4f) {
    primary_axis_2d = Eigen::Vector2f::UnitX();
  } else {
    primary_axis_2d.normalize();
  }

  // Gripper Y aligns with the object length
  Eigen::Vector3f y_gripper(primary_axis_2d.x(), primary_axis_2d.y(), 0.0f);
  y_gripper.normalize();

  // Approach Z constrained downward (into table)
  Eigen::Vector3f z_gripper(0.0f, 0.0f, -1.0f);
  // Finger opening/closing axis (X) perpendicular to length (aligned with object width)
  Eigen::Vector3f x_gripper = y_gripper.cross(z_gripper).normalized();
  // Ensure right-handed orthogonal frame: x cross y = z
  y_gripper = z_gripper.cross(x_gripper).normalized();

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

bool IrregularObjectPoseEstimator::detect_red_object_2d(
  const cv::Mat & bgr_img,
  cv::Point2f & centroid_2d,
  cv::RotatedRect & min_rect,
  cv::Mat & debug_img)
{
  if (bgr_img.empty()) {
    return false;
  }

  debug_img = bgr_img.clone();

  cv::Mat hsv_img;
  cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);

  // Two red ranges in HSV (hue wraps around 0 and 180)
  cv::Mat mask1, mask2, red_mask;
  cv::inRange(
    hsv_img,
    cv::Scalar(hsv_red_h_low_1_, hsv_red_s_min_, hsv_red_v_min_),
    cv::Scalar(hsv_red_h_high_1_, 255, 255), mask1);
  cv::inRange(
    hsv_img,
    cv::Scalar(hsv_red_h_low_2_, hsv_red_s_min_, hsv_red_v_min_),
    cv::Scalar(hsv_red_h_high_2_, 255, 255), mask2);

  red_mask = mask1 | mask2;

  // Morphological opening and closing to clean noise
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    return false;
  }

  // Find largest contour
  double max_area = 0.0;
  size_t max_idx = 0;
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > max_area) {
      max_area = area;
      max_idx = i;
    }
  }

  // Minimum contour area threshold (at 2.5m height, 15x8cm object is ~500-1500 pixels)
  if (max_area < 50.0) {
    return false;
  }

  cv::Moments m = cv::moments(contours[max_idx]);
  if (m.m00 <= 1e-4) {
    return false;
  }

  centroid_2d.x = static_cast<float>(m.m10 / m.m00);
  centroid_2d.y = static_cast<float>(m.m01 / m.m00);

  min_rect = cv::minAreaRect(contours[max_idx]);

  // Annotate debug image with rotated bounding box and centroid
  cv::Point2f rect_points[4];
  min_rect.points(rect_points);
  for (int j = 0; j < 4; j++) {
    cv::line(debug_img, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
  }

  cv::circle(debug_img, centroid_2d, 5, cv::Scalar(0, 0, 255), -1);
  cv::circle(debug_img, centroid_2d, 8, cv::Scalar(255, 255, 255), 2);

  char text_buf[128];
  std::snprintf(text_buf, sizeof(text_buf), "RED TARGET [%.0f, %.0f]", centroid_2d.x, centroid_2d.y);
  cv::putText(
    debug_img, text_buf, cv::Point(centroid_2d.x - 60, centroid_2d.y - 15),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

  return true;
}

void IrregularObjectPoseEstimator::rgb_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Point2f c_2d;
  cv::RotatedRect m_rect;
  cv::Mat debug_img;
  bool detected = detect_red_object_2d(cv_ptr->image, c_2d, m_rect, debug_img);

  {
    std::lock_guard<std::mutex> lock(rgb_mutex_);
    latest_rgb_image_ = cv_ptr->image.clone();
    latest_red_2d_detected_ = detected;
    latest_red_2d_centroid_ = c_2d;
    latest_rgb_stamp_ = msg->header.stamp;
  }

  if (pub_annotated_image_ && pub_annotated_image_->is_activated()) {
    std_msgs::msg::Header header = msg->header;
    sensor_msgs::msg::Image::SharedPtr out_msg =
      cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg();
    pub_annotated_image_->publish(*out_msg);
  }
}

bool IrregularObjectPoseEstimator::is_cluster_red(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr /*cluster*/,
  const geometry_msgs::msg::Pose & cluster_pose,
  const cv::Mat & rgb_img,
  double * dist_2d_out)
{
  if (dist_2d_out) {
    *dist_2d_out = 1e9;
  }

  if (!enable_color_filtering_) {
    return true;
  }

  if (rgb_img.empty()) {
    return false;
  }

  // Camera intrinsics: 640x480, HFOV=1.2 rad -> fx = fy = 467.74, cx = 320, cy = 240
  const double fx = 467.74;
  const double fy = 467.74;
  const double cx = 320.0;
  const double cy = 240.0;

  double u_proj = -1.0, v_proj = -1.0;

  // Project 3D centroid from target_frame_ ("world") into depth_camera_optical_frame
  if (tf_buffer_) {
    try {
      geometry_msgs::msg::PoseStamped pose_world, pose_cam;
      pose_world.header.frame_id = target_frame_;
      pose_world.header.stamp = rclcpp::Time(0, 0);
      pose_world.pose = cluster_pose;

      if (tf_buffer_->canTransform("depth_camera_optical_frame", target_frame_, tf2::TimePointZero)) {
        auto transform = tf_buffer_->lookupTransform(
          "depth_camera_optical_frame", target_frame_,
          tf2::TimePointZero, std::chrono::milliseconds(50));
        tf2::doTransform(pose_world, pose_cam, transform);

        double z_c = pose_cam.pose.position.z;
        if (z_c > 0.1) {
          u_proj = (fx * pose_cam.pose.position.x / z_c) + cx;
          v_proj = (fy * pose_cam.pose.position.y / z_c) + cy;
        }
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF projection exception: %s", ex.what());
    }
  }

  // Ensure projected center is inside the image bounds
  if (u_proj < 5.0 || u_proj >= rgb_img.cols - 5 ||
      v_proj < 5.0 || v_proj >= rgb_img.rows - 5)
  {
    return false;
  }

  // Direct RGB pixel color sampling in a 9x9 neighborhood around projected center
  int u_i = static_cast<int>(std::round(u_proj));
  int v_i = static_cast<int>(std::round(v_proj));

  int r_sum = 0, g_sum = 0, b_sum = 0, count = 0;
  for (int dy = -4; dy <= 4; ++dy) {
    for (int dx = -4; dx <= 4; ++dx) {
      cv::Vec3b bgr = rgb_img.at<cv::Vec3b>(v_i + dy, u_i + dx);
      b_sum += bgr[0];
      g_sum += bgr[1];
      r_sum += bgr[2];
      count++;
    }
  }
  double r_avg = static_cast<double>(r_sum) / count;
  double g_avg = static_cast<double>(g_sum) / count;
  double b_avg = static_cast<double>(b_sum) / count;

  // Strict red channel verification: Red must be bright and dominant over Blue and Green.
  // The blue obstacle has B > 120, R < 50; the table is grey (R ~ G ~ B).
  // Only the red object satisfies R > 80 && R > 1.30 * B && R > 1.30 * G.
  bool color_is_red = (r_avg > 80.0 && r_avg > 1.30 * b_avg && r_avg > 1.30 * g_avg);
  if (!color_is_red) {
    return false;
  }

  // Correlate with 2D red contour detection from overhead camera if available
  if (latest_red_2d_detected_) {
    double dist_pix = std::hypot(u_proj - latest_red_2d_centroid_.x, v_proj - latest_red_2d_centroid_.y);
    if (dist_2d_out) {
      *dist_2d_out = dist_pix;
    }
    // True red object projects within 15-20 pixels of 2D contour; blue obstacle is >50px away
    if (dist_pix > 35.0) {
      return false;
    }
  } else if (dist_2d_out) {
    *dist_2d_out = 0.0;
  }

  return true;
}

void IrregularObjectPoseEstimator::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  last_cloud_stamp_ = msg->header.stamp;

  // Transform pointcloud to target_frame_ ("world") BEFORE any math
  sensor_msgs::msg::PointCloud2 transformed_msg;
  if (tf_buffer_ && msg->header.frame_id != target_frame_) {
    try {
      auto transform = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id,
        tf2::TimePointZero, std::chrono::milliseconds(50));
      tf2::doTransform(*msg, transformed_msg, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not transform pointcloud from %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }
  } else {
    transformed_msg = *msg;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_msg, *input_cloud);

  // Filter out NaN / Inf points from camera stream
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *clean_cloud, nan_indices);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Pointcloud received: raw=%zu, clean=%zu, min_thresh=%d",
    input_cloud->size(), clean_cloud->size(), min_valid_points_);

  if (static_cast<int>(clean_cloud->size()) < min_valid_points_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received point cloud with only %zu valid points (minimum threshold: %d)",
      clean_cloud->size(), min_valid_points_);
    tracked_objects_count_ = 0;
    last_processing_latency_ms_ = 0.0;
    return;
  }

  // Filter point cloud to valid manipulation workspace on the table:
  // Discards overhead robot links (Z > 0.35m), ground/floor, and outside table boundaries
  pcl::PointCloud<pcl::PointXYZ>::Ptr workspace_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  workspace_cloud->reserve(clean_cloud->size());
  for (const auto & pt : clean_cloud->points) {
    if (pt.x >= workspace_min_x_ && pt.x <= workspace_max_x_ &&
        pt.y >= workspace_min_y_ && pt.y <= workspace_max_y_ &&
        pt.z >= workspace_min_z_ && pt.z <= workspace_max_z_) {
      // Exclude points on or directly around Arm 1 base pedestal (-0.60, 0.0)
      double dist_arm1 = std::hypot(pt.x - (-0.60), pt.y - 0.0);
      if (dist_arm1 < 0.15) {
        continue;
      }
      workspace_cloud->points.push_back(pt);
    }
  }
  workspace_cloud->width = workspace_cloud->points.size();
  workspace_cloud->height = 1;
  workspace_cloud->is_dense = true;

  if (static_cast<int>(workspace_cloud->size()) < min_valid_points_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Workspace point cloud has only %zu valid points (minimum threshold: %d)",
      workspace_cloud->size(), min_valid_points_);
    tracked_objects_count_ = 0;
    last_processing_latency_ms_ = 0.0;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  segment_table_plane(workspace_cloud, non_table_cloud, last_inlier_pct_);

  auto clusters = extract_clusters(non_table_cloud);
  tracked_objects_count_ = static_cast<int>(clusters.size());

  // Snapshot latest RGB image thread-safely
  cv::Mat current_rgb;
  {
    std::lock_guard<std::mutex> lock(rgb_mutex_);
    if (!latest_rgb_image_.empty()) {
      current_rgb = latest_rgb_image_.clone();
    }
  }

  // Evaluate each cluster for pose and color
  struct ClusterInfo {
    size_t cluster_idx;
    geometry_msgs::msg::Pose pose;
    bool is_red;
    double dist_2d_to_red;
  };
  std::vector<ClusterInfo> detected_clusters;

  for (size_t i = 0; i < clusters.size(); ++i) {
    geometry_msgs::msg::Pose target_pose;
    if (compute_cluster_pose(clusters[i], target_pose)) {
      double dist_2d = 1e9;
      bool is_red = is_cluster_red(clusters[i], target_pose, current_rgb, &dist_2d);
      detected_clusters.push_back({i, target_pose, is_red, dist_2d});
    }
  }

  // Find the single best red cluster (smallest distance to 2D red contour centroid)
  int best_red_idx = -1;
  double min_dist_2d = 1e9;
  for (size_t i = 0; i < detected_clusters.size(); ++i) {
    if (detected_clusters[i].is_red) {
      if (detected_clusters[i].dist_2d_to_red < min_dist_2d) {
        min_dist_2d = detected_clusters[i].dist_2d_to_red;
        best_red_idx = static_cast<int>(i);
      }
    }
  }

  // Swap best red cluster to index 0 so red target is always prioritized
  if (best_red_idx > 0) {
    std::swap(detected_clusters[0], detected_clusters[best_red_idx]);
  }
  red_target_detected_ = (!detected_clusters.empty() && detected_clusters[0].is_red);

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = msg->header.stamp;
  pose_array.header.frame_id = target_frame_;

  visualization_msgs::msg::MarkerArray marker_array;
  pcl::PointCloud<pcl::PointXYZ> combined_object_cloud;

  for (size_t i = 0; i < detected_clusters.size(); ++i) {
    const auto & c_info = detected_clusters[i];
    const auto & target_pose = c_info.pose;
    bool is_red = c_info.is_red;
    size_t orig_idx = c_info.cluster_idx;

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

    // If this is the red target, also broadcast dedicated "red_object_target" frame & publish red_object_pose
    if (is_red) {
      geometry_msgs::msg::TransformStamped red_tf;
      red_tf.header.stamp = msg->header.stamp;
      red_tf.header.frame_id = target_frame_;
      red_tf.child_frame_id = "red_object_target";
      red_tf.transform = tf_msg.transform;
      tf_broadcaster_->sendTransform(red_tf);

      if (pub_red_object_pose_ && pub_red_object_pose_->is_activated()) {
        geometry_msgs::msg::PoseStamped red_pose_msg;
        red_pose_msg.header.stamp = msg->header.stamp;
        red_pose_msg.header.frame_id = target_frame_;
        red_pose_msg.pose = target_pose;
        pub_red_object_pose_->publish(red_pose_msg);
      }
    }

    // Compute Euler angles (Roll, Pitch, Yaw)
    tf2::Quaternion q(
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z,
      target_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Distinct console log of perception solution
    if (is_red) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "==> [RGB-D SOLVED] RED TARGET OBJECT: Pos=[X: %+.3f, Y: %+.3f, Z: %+.3f] m | "
        "Yaw=%+.1f deg (%+.3f rad) | ClusterPoints=%zu",
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        yaw * 180.0 / M_PI, yaw, clusters[orig_idx]->size());
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "    [RGB-D SOLVED] Secondary / Obstacle %zu: Pos=[X: %+.3f, Y: %+.3f, Z: %+.3f] m | "
        "Yaw=%+.1f deg (%+.3f rad) | ClusterPoints=%zu",
        i + 1,
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        yaw * 180.0 / M_PI, yaw, clusters[orig_idx]->size());
    }

    // 1. Centroid Sphere Marker
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.stamp = msg->header.stamp;
    sphere_marker.header.frame_id = target_frame_;
    sphere_marker.ns = "object_centroids";
    sphere_marker.id = static_cast<int>(i);
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose = target_pose;
    sphere_marker.scale.x = 0.045;
    sphere_marker.scale.y = 0.045;
    sphere_marker.scale.z = 0.045;
    if (is_red) {
      // Bright red for target object
      sphere_marker.color.r = 1.0f;
      sphere_marker.color.g = 0.05f;
      sphere_marker.color.b = 0.05f;
      sphere_marker.color.a = 0.95f;
    } else {
      // Deep blue for obstacles/secondary
      sphere_marker.color.r = 0.1f;
      sphere_marker.color.g = 0.35f;
      sphere_marker.color.b = 1.0f;
      sphere_marker.color.a = 0.9f;
    }
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker_array.markers.push_back(sphere_marker);

    // 2. Grasp Approach Arrow Marker
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.stamp = msg->header.stamp;
    arrow_marker.header.frame_id = target_frame_;
    arrow_marker.ns = "grasp_axes";
    arrow_marker.id = static_cast<int>(i);
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.pose = target_pose;
    arrow_marker.scale.x = 0.12;  // shaft length
    arrow_marker.scale.y = 0.015; // shaft diameter
    arrow_marker.scale.z = 0.015; // head diameter
    if (is_red) {
      arrow_marker.color.r = 1.0f;
      arrow_marker.color.g = 0.85f;
      arrow_marker.color.b = 0.0f;
      arrow_marker.color.a = 1.0f;
    } else {
      arrow_marker.color.r = 0.3f;
      arrow_marker.color.g = 0.7f;
      arrow_marker.color.b = 1.0f;
      arrow_marker.color.a = 0.8f;
    }
    arrow_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker_array.markers.push_back(arrow_marker);

    // 3. 3D Text Billboard Marker above object
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.stamp = msg->header.stamp;
    text_marker.header.frame_id = target_frame_;
    text_marker.ns = "object_labels";
    text_marker.id = static_cast<int>(i);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = target_pose.position.x;
    text_marker.pose.position.y = target_pose.position.y;
    text_marker.pose.position.z = target_pose.position.z + 0.12;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.035;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;

    char label_buf[128];
    if (is_red) {
      std::snprintf(
        label_buf, sizeof(label_buf),
        "[RED TARGET]\n[%.2f, %.2f, %.2f]\nYaw: %+.1f°",
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        yaw * 180.0 / M_PI);
    } else {
      std::snprintf(
        label_buf, sizeof(label_buf),
        "[OBSTACLE]\n[%.2f, %.2f, %.2f]",
        target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }
    text_marker.text = label_buf;
    text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker_array.markers.push_back(text_marker);

    combined_object_cloud += *clusters[orig_idx];
  }

  if (pub_target_poses_ && pub_target_poses_->is_activated()) {
    pub_target_poses_->publish(pose_array);
  }

  if (pub_target_markers_ && pub_target_markers_->is_activated()) {
    pub_target_markers_->publish(marker_array);
  }

  if (pub_object_cloud_ && pub_object_cloud_->is_activated()) {
    sensor_msgs::msg::PointCloud2 object_cloud_msg;
    pcl::toROSMsg(combined_object_cloud, object_cloud_msg);
    object_cloud_msg.header.stamp = msg->header.stamp;
    object_cloud_msg.header.frame_id = target_frame_;
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
