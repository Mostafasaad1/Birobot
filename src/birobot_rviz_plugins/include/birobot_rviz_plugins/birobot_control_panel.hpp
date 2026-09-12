// Copyright 2026 Birobot Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BIROBOT_RVIZ_PLUGINS__BIROBOT_CONTROL_PANEL_HPP_
#define BIROBOT_RVIZ_PLUGINS__BIROBOT_CONTROL_PANEL_HPP_

#include <memory>
#include <string>

#include <QWidget>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QTabWidget>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <birobot_interfaces/srv/randomize_object.hpp>

namespace birobot_rviz_plugins
{

class BirobotControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BirobotControlPanel(QWidget * parent = nullptr);
  ~BirobotControlPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void onPlanAndExecuteClicked();
  void onRandomizeClicked();
  void onApplyPoseClicked();
  void onStopClicked();
  void onMoveHomeClicked();
  void onZoneChanged(int index);
  void updateStatusFromSignal(const QString & status_text);
  void updateObjectPoseFromSignal(double x, double y, double z, double yaw, const QString & desc);

Q_SIGNALS:
  void missionStatusReceived(const QString & status_text);
  void objectPoseUpdated(double x, double y, double z, double yaw, const QString & desc);

private:
  void buildUi();
  void applyMoveItStyle();
  std::string getSelectedZoneCode() const;

  // ROS 2 communication handles
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<birobot_interfaces::srv::RandomizeObject>::SharedPtr client_randomize_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_trigger_handover_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_reset_mission_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mission_status_;

  // UI elements: Tabs
  QTabWidget * tab_widget_;

  // Column 1: Commands
  QGroupBox * grp_commands_;
  QPushButton * btn_plan_exec_;
  QPushButton * btn_randomize_;
  QPushButton * btn_apply_pose_;
  QPushButton * btn_stop_;
  QPushButton * btn_move_home_;

  // Column 2: Query
  QGroupBox * grp_query_;
  QComboBox * combo_group_;
  QComboBox * combo_zone_;
  QComboBox * combo_start_;
  QComboBox * combo_goal_;
  QComboBox * combo_state_;

  // Column 3: Options
  QGroupBox * grp_options_;
  QDoubleSpinBox * spin_time_;
  QSpinBox * spin_attempts_;
  QDoubleSpinBox * spin_velocity_;
  QDoubleSpinBox * spin_x_;
  QDoubleSpinBox * spin_y_;
  QDoubleSpinBox * spin_z_;
  QDoubleSpinBox * spin_yaw_;
};

}  // namespace birobot_rviz_plugins

#endif  // BIROBOT_RVIZ_PLUGINS__BIROBOT_CONTROL_PANEL_HPP_
