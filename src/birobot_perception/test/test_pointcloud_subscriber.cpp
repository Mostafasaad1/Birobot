#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "birobot_perception/irregular_object_pose_estimator.hpp"

class PointCloudSubscriberTest : public ::testing::Test
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

TEST_F(PointCloudSubscriberTest, NodeLifecycleStateTransitions)
{
  auto node = std::make_shared<birobot_perception::IrregularObjectPoseEstimator>();

  EXPECT_EQ(node->get_current_state().label(), "unconfigured");

  EXPECT_EQ(node->configure().label(), "inactive");
  EXPECT_EQ(node->activate().label(), "active");
  EXPECT_EQ(node->deactivate().label(), "inactive");
  EXPECT_EQ(node->cleanup().label(), "unconfigured");
}

TEST_F(PointCloudSubscriberTest, ValidPointCloudMessageBuffer)
{
  auto node = std::make_shared<birobot_perception::IrregularObjectPoseEstimator>();
  node->configure();
  node->activate();

  // Create synthetic point cloud with 1200 points
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 1200;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = 0.001f * i;
    cloud.points[i].y = 0.002f * i;
    cloud.points[i].z = 0.8f;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";

  auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_msg);

  // Invoke callback directly
  EXPECT_NO_THROW(node->pointcloud_callback(msg_ptr));
}
