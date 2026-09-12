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

TEST_F(PCAPoseEstimationTest, RedColor2DDetection)
{
  auto node = std::make_shared<birobot_perception::IrregularObjectPoseEstimator>();
  node->configure();

  // Create a 640x480 neutral background synthetic image
  cv::Mat test_img(480, 640, CV_8UC3, cv::Scalar(160, 160, 160));

  // Draw a bright red rectangle centered at (300, 200) with size 100x50
  cv::rectangle(test_img, cv::Rect(250, 175, 100, 50), cv::Scalar(20, 20, 220), -1);

  cv::Point2f centroid_2d;
  cv::RotatedRect min_rect;
  cv::Mat debug_img;
  bool detected = node->detect_red_object_2d(test_img, centroid_2d, min_rect, debug_img);

  EXPECT_TRUE(detected);
  EXPECT_NEAR(centroid_2d.x, 300.0f, 2.0f);
  EXPECT_NEAR(centroid_2d.y, 200.0f, 2.0f);
}
