#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "birobot_perception/irregular_object_pose_estimator.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class RANSACFilteringTest : public ::testing::Test
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

TEST_F(RANSACFilteringTest, TablePlaneSegmentationAndClustering)
{
  auto node = std::make_shared<birobot_perception::IrregularObjectPoseEstimator>();
  node->configure();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 1. Generate 2000 table plane points at z = 0.0 with slight noise
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> xy_dist(-0.5f, 0.5f);
  std::uniform_real_distribution<float> z_noise(-0.002f, 0.002f);

  for (int i = 0; i < 2000; ++i) {
    pcl::PointXYZ p;
    p.x = xy_dist(gen);
    p.y = xy_dist(gen);
    p.z = z_noise(gen);
    input_cloud->points.push_back(p);
  }

  // 2. Add an isolated object cluster (200 points) centered at (0.2, 0.1, 0.15)
  std::uniform_real_distribution<float> obj_dist(-0.03f, 0.03f);
  for (int i = 0; i < 200; ++i) {
    pcl::PointXYZ p;
    p.x = 0.20f + obj_dist(gen);
    p.y = 0.10f + obj_dist(gen);
    p.z = 0.15f + obj_dist(gen);
    input_cloud->points.push_back(p);
  }

  input_cloud->width = input_cloud->points.size();
  input_cloud->height = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr non_table_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  double inlier_pct = 0.0;

  bool seg_success = node->segment_table_plane(input_cloud, non_table_cloud, inlier_pct);

  EXPECT_TRUE(seg_success);
  EXPECT_GT(inlier_pct, 80.0);  // ~2000 out of 2200 points in table plane
  EXPECT_NEAR(non_table_cloud->size(), 200, 30);

  auto clusters = node->extract_clusters(non_table_cloud);
  EXPECT_GE(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0]->size(), 200, 30);
}
