#include <gtest/gtest.h>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "birobot_perception/irregular_object_pose_estimator.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PCAPoseEstimationTest : public ::testing::Test
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

TEST_F(PCAPoseEstimationTest, CentroidAndQuaternionCalculation)
{
  auto node = std::make_shared<birobot_perception::IrregularObjectPoseEstimator>();
  node->configure();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

  // Create an elongated object cluster along X-axis centered at (0.35, -0.20, 0.85)
  float center_x = 0.35f;
  float center_y = -0.20f;
  float center_z = 0.85f;

  for (float dx = -0.10f; dx <= 0.10f; dx += 0.01f) {
    for (float dy = -0.02f; dy <= 0.02f; dy += 0.01f) {
      for (float dz = -0.02f; dz <= 0.02f; dz += 0.01f) {
        pcl::PointXYZ p;
        p.x = center_x + dx;
        p.y = center_y + dy;
        p.z = center_z + dz;
        cluster->points.push_back(p);
      }
    }
  }

  cluster->width = cluster->points.size();
  cluster->height = 1;

  geometry_msgs::msg::Pose pose;
  bool success = node->compute_cluster_pose(cluster, pose);

  EXPECT_TRUE(success);

  // Validate centroid within ±5 mm (0.005 m)
  EXPECT_NEAR(pose.position.x, center_x, 0.005);
  EXPECT_NEAR(pose.position.y, center_y, 0.005);
  EXPECT_NEAR(pose.position.z, center_z, 0.005);

  // Validate quaternion normalization
  double q_norm = std::sqrt(
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);

  EXPECT_NEAR(q_norm, 1.0, 1e-4);
}
