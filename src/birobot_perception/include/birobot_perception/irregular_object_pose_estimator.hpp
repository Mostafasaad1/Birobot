#ifndef BIROBOT_PERCEPTION__IRREGULAR_OBJECT_POSE_ESTIMATOR_HPP_
#define BIROBOT_PERCEPTION__IRREGULAR_OBJECT_POSE_ESTIMATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace birobot_perception
{

class IrregularObjectPoseEstimator : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit IrregularObjectPoseEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~IrregularObjectPoseEstimator() = default;

  // Lifecycle Interface Callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Processing & Algorithm API (Exposed for unit testing & isolation harnesses)
  bool segment_table_plane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_cloud,
    double & inlier_pct);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extract_clusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_cloud);

  bool compute_cluster_pose(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
    geometry_msgs::msg::Pose & pose);

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  int get_tracked_objects_count() const { return tracked_objects_count_; }
  double get_last_processing_latency_ms() const { return last_processing_latency_ms_; }
  double get_last_inlier_pct() const { return last_inlier_pct_; }

private:
  void publish_diagnostics();

  // ROS 2 Subscriptions and Publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_cloud_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_target_poses_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_diagnostics_;

  // Parameters
  std::string input_cloud_topic_;
  std::string target_frame_;
  std::string target_tf_prefix_;
  double ransac_distance_threshold_;
  int ransac_max_iterations_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  int max_tracked_objects_;
  double diagnostic_rate_hz_;
  int min_valid_points_;

  // Observability & Diagnostic Metrics
  double last_processing_latency_ms_;
  double last_inlier_pct_;
  int tracked_objects_count_;
  rclcpp::Time last_cloud_stamp_;
};

}  // namespace birobot_perception

#endif  // BIROBOT_PERCEPTION__IRREGULAR_OBJECT_POSE_ESTIMATOR_HPP_
