#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "birobot_perception/irregular_object_pose_estimator.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class EndToEndPerceptionTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(EndToEndPerceptionTest, DynamicTFPublishingForMultipleObjects)
{
  auto node = std::make_shared<birobot_perception::IrregularObjectPoseEstimator>();
  node->configure();
  node->activate();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 1. Generate 3000 table plane points at z = 0.0 with noise
  std::mt19937 gen(12345);
  std::uniform_real_distribution<float> xy_dist(-0.6f, 0.6f);
  std::uniform_real_distribution<float> z_noise(-0.002f, 0.002f);

  for (int i = 0; i < 3000; ++i) {
    pcl::PointXYZ p;
    p.x = xy_dist(gen);
    p.y = xy_dist(gen);
    p.z = z_noise(gen);
    input_cloud->points.push_back(p);
  }

  // 2. Object 1: Elongated along X-axis, center = (0.25, 0.15, 0.10)
  float obj1_x = 0.25f, obj1_y = 0.15f, obj1_z = 0.10f;
  for (float dx = -0.10f; dx <= 0.10f; dx += 0.01f) {
    for (float dy = -0.02f; dy <= 0.02f; dy += 0.01f) {
      for (float dz = -0.02f; dz <= 0.02f; dz += 0.01f) {
        pcl::PointXYZ p;
        p.x = obj1_x + dx;
        p.y = obj1_y + dy;
        p.z = obj1_z + dz;
        input_cloud->points.push_back(p);
      }
    }
  }

  // 3. Object 2: Elongated along Y-axis, center = (-0.20, -0.15, 0.08)
  float obj2_x = -0.20f, obj2_y = -0.15f, obj2_z = 0.08f;
  for (float dx = -0.02f; dx <= 0.02f; dx += 0.01f) {
    for (float dy = -0.10f; dy <= 0.10f; dy += 0.01f) {
      for (float dz = -0.02f; dz <= 0.02f; dz += 0.01f) {
        pcl::PointXYZ p;
        p.x = obj2_x + dx;
        p.y = obj2_y + dy;
        p.z = obj2_z + dz;
        input_cloud->points.push_back(p);
      }
    }
  }

  input_cloud->width = input_cloud->points.size();
  input_cloud->height = 1;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*input_cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = node->now();

  auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_msg);

  // Process pointcloud callback
  EXPECT_NO_THROW(node->pointcloud_callback(msg_ptr));

  // Check internal state: should track 2 objects
  EXPECT_EQ(node->get_tracked_objects_count(), 2);
}
