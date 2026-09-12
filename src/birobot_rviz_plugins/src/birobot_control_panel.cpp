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

#include "birobot_rviz_plugins/birobot_control_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace birobot_rviz_plugins
{

BirobotControlPanel::BirobotControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  tab_widget_(nullptr),
  grp_commands_(nullptr),
  btn_plan_exec_(nullptr),
  btn_randomize_(nullptr),
  btn_apply_pose_(nullptr),
  btn_stop_(nullptr),
  btn_move_home_(nullptr),
  grp_query_(nullptr),
  combo_group_(nullptr),
  combo_zone_(nullptr),
  combo_start_(nullptr),
  combo_goal_(nullptr),
  combo_state_(nullptr),
  grp_options_(nullptr),
  spin_time_(nullptr),
  spin_attempts_(nullptr),
  spin_velocity_(nullptr),
  spin_x_(nullptr),
  spin_y_(nullptr),
  spin_z_(nullptr),
  spin_yaw_(nullptr)
{
  buildUi();
  applyMoveItStyle();

  connect(this, &BirobotControlPanel::missionStatusReceived,
          this, &BirobotControlPanel::updateStatusFromSignal);
  connect(this, &BirobotControlPanel::objectPoseUpdated,
          this, &BirobotControlPanel::updateObjectPoseFromSignal);
}

BirobotControlPanel::~BirobotControlPanel() = default;

void BirobotControlPanel::buildUi()
{
  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(4, 4, 4, 4);
  root_layout->setSpacing(4);

  tab_widget_ = new QTabWidget(this);

  // ═══════════════════════════════════════════════════════════════════════════
  // Tab 0: Context
  // ═══════════════════════════════════════════════════════════════════════════
  auto * tab_context = new QWidget(this);
  auto * layout_ctx = new QVBoxLayout(tab_context);
  layout_ctx->setContentsMargins(8, 8, 8, 8);
  layout_ctx->setSpacing(6);

  auto * lbl_ctx = new QLabel(
    tr("Birobot Dual-Arm Workcell Context\n\n"
       "Planning Group: dual_arms (arm_1 + arm_2)\n"
       "Table Bounds: X in [-0.80, +0.80] m, Y in [-0.40, +0.40] m\n"
       "Ground Pick Zone (past table): X in [-1.08, -0.90] m, Z = 0.100 m\n"
       "Arm 1 Pedestal: (-0.600, 0.000, 0.050) m\n"
       "Arm 2 Pedestal: (+0.600, 0.000, 0.050) m\n"
       "Depth Camera: (-0.850, 0.000, 2.200) m"),
    tab_context);
  lbl_ctx->setWordWrap(true);
  layout_ctx->addWidget(lbl_ctx);
  layout_ctx->addStretch();

  tab_widget_->addTab(tab_context, tr("Context"));

  // ═══════════════════════════════════════════════════════════════════════════
  // Tab 1: Planning (MoveIt 3-Column Design Language)
  // ═══════════════════════════════════════════════════════════════════════════
  auto * tab_planning = new QWidget(this);
  auto * layout_columns = new QHBoxLayout(tab_planning);
  layout_columns->setContentsMargins(6, 6, 6, 6);
  layout_columns->setSpacing(8);

  // ───────────────────────────────────────────────────────────────────────────
  // Column 1: Commands
  // ───────────────────────────────────────────────────────────────────────────
  grp_commands_ = new QGroupBox(tr("Commands"), tab_planning);
  grp_commands_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  auto * layout_commands = new QVBoxLayout(grp_commands_);
  layout_commands->setContentsMargins(8, 14, 8, 8);
  layout_commands->setSpacing(6);

  btn_plan_exec_ = new QPushButton(tr("Plan & Execute"), grp_commands_);
  btn_randomize_ = new QPushButton(tr("Randomize Object"), grp_commands_);
  btn_apply_pose_ = new QPushButton(tr("Apply Pose"), grp_commands_);
  btn_stop_ = new QPushButton(tr("Stop"), grp_commands_);
  btn_stop_->setEnabled(false);
  btn_move_home_ = new QPushButton(tr("Move Home"), grp_commands_);

  btn_plan_exec_->setMinimumHeight(24);
  btn_randomize_->setMinimumHeight(24);
  btn_apply_pose_->setMinimumHeight(24);
  btn_stop_->setMinimumHeight(24);
  btn_move_home_->setMinimumHeight(24);

  layout_commands->addWidget(btn_plan_exec_);
  layout_commands->addWidget(btn_randomize_);
  layout_commands->addWidget(btn_apply_pose_);
  layout_commands->addWidget(btn_stop_);
  layout_commands->addSpacing(16);
  layout_commands->addWidget(btn_move_home_);
  layout_commands->addStretch();

  layout_columns->addWidget(grp_commands_, 1);

  // ───────────────────────────────────────────────────────────────────────────
  // Column 2: Query
  // ───────────────────────────────────────────────────────────────────────────
  grp_query_ = new QGroupBox(tr("Query"), tab_planning);
  grp_query_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  auto * layout_query = new QVBoxLayout(grp_query_);
  layout_query->setContentsMargins(8, 14, 8, 8);
  layout_query->setSpacing(6);

  // Planning Group
  auto * lbl_group = new QLabel(tr("Planning Group:"), grp_query_);
  combo_group_ = new QComboBox(grp_query_);
  combo_group_->addItem(tr("dual_arms"));
  combo_group_->addItem(tr("arm_1"));
  combo_group_->addItem(tr("arm_2"));
  combo_group_->setMinimumHeight(24);

  layout_query->addWidget(lbl_group);
  layout_query->addWidget(combo_group_);

  // Spawn Zone
  auto * lbl_zone = new QLabel(tr("Spawn Zone:"), grp_query_);
  combo_zone_ = new QComboBox(grp_query_);
  combo_zone_->addItem(tr("other_side"), QString("other_side"));
  combo_zone_->addItem(tr("front"), QString("front"));
  combo_zone_->addItem(tr("all"), QString("all"));
  combo_zone_->addItem(tr("custom"), QString("custom"));
  combo_zone_->setMinimumHeight(24);

  layout_query->addWidget(lbl_zone);
  layout_query->addWidget(combo_zone_);

  // Start State
  auto * lbl_start = new QLabel(tr("Start State:"), grp_query_);
  combo_start_ = new QComboBox(grp_query_);
  combo_start_->addItem(tr("<current>"));
  combo_start_->addItem(tr("<home>"));
  combo_start_->setMinimumHeight(24);

  layout_query->addWidget(lbl_start);
  layout_query->addWidget(combo_start_);

  // Goal State
  auto * lbl_goal = new QLabel(tr("Goal State:"), grp_query_);
  combo_goal_ = new QComboBox(grp_query_);
  combo_goal_->addItem(tr("<current>"));
  combo_goal_->addItem(tr("<handover>"));
  combo_goal_->addItem(tr("<table_deposit>"));
  combo_goal_->addItem(tr("<bin_drop>"));
  combo_goal_->setMinimumHeight(24);

  layout_query->addWidget(lbl_goal);
  layout_query->addWidget(combo_goal_);

  // Mission State
  auto * lbl_state = new QLabel(tr("Mission State:"), grp_query_);
  combo_state_ = new QComboBox(grp_query_);
  combo_state_->addItem(tr("<current: IDLE>"));
  combo_state_->setMinimumHeight(24);

  layout_query->addWidget(lbl_state);
  layout_query->addWidget(combo_state_);
  layout_query->addStretch();

  layout_columns->addWidget(grp_query_, 1);

  // ───────────────────────────────────────────────────────────────────────────
  // Column 3: Options
  // ───────────────────────────────────────────────────────────────────────────
  grp_options_ = new QGroupBox(tr("Options"), tab_planning);
  grp_options_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  auto * layout_options = new QGridLayout(grp_options_);
  layout_options->setContentsMargins(8, 14, 8, 8);
  layout_options->setHorizontalSpacing(6);
  layout_options->setVerticalSpacing(6);

  spin_time_ = new QDoubleSpinBox(grp_options_);
  spin_time_->setValue(5.0);
  spin_time_->setSingleStep(1.0);
  spin_time_->setDecimals(1);
  spin_time_->setMinimumHeight(24);

  spin_attempts_ = new QSpinBox(grp_options_);
  spin_attempts_->setRange(1, 100);
  spin_attempts_->setValue(10);
  spin_attempts_->setMinimumHeight(24);

  spin_velocity_ = new QDoubleSpinBox(grp_options_);
  spin_velocity_->setRange(0.01, 1.00);
  spin_velocity_->setSingleStep(0.05);
  spin_velocity_->setValue(0.10);
  spin_velocity_->setDecimals(2);
  spin_velocity_->setMinimumHeight(24);

  spin_x_ = new QDoubleSpinBox(grp_options_);
  spin_x_->setRange(-1.30, 0.60);
  spin_x_->setSingleStep(0.05);
  spin_x_->setValue(-0.950);
  spin_x_->setDecimals(3);
  spin_x_->setMinimumHeight(24);

  spin_y_ = new QDoubleSpinBox(grp_options_);
  spin_y_->setRange(-0.60, 0.60);
  spin_y_->setSingleStep(0.05);
  spin_y_->setValue(0.000);
  spin_y_->setDecimals(3);
  spin_y_->setMinimumHeight(24);

  spin_z_ = new QDoubleSpinBox(grp_options_);
  spin_z_->setRange(0.00, 0.50);
  spin_z_->setSingleStep(0.01);
  spin_z_->setValue(0.100);
  spin_z_->setDecimals(3);
  spin_z_->setMinimumHeight(24);

  spin_yaw_ = new QDoubleSpinBox(grp_options_);
  spin_yaw_->setRange(-180.0, 180.0);
  spin_yaw_->setSingleStep(5.0);
  spin_yaw_->setValue(0.0);
  spin_yaw_->setDecimals(1);
  spin_yaw_->setMinimumHeight(24);

  int r = 0;
  layout_options->addWidget(new QLabel(tr("Planning Time (s):"), grp_options_), r, 0);
  layout_options->addWidget(spin_time_, r, 1);
  r++;
  layout_options->addWidget(new QLabel(tr("Planning Attempts:"), grp_options_), r, 0);
  layout_options->addWidget(spin_attempts_, r, 1);
  r++;
  layout_options->addWidget(new QLabel(tr("Velocity Scaling:"), grp_options_), r, 0);
  layout_options->addWidget(spin_velocity_, r, 1);
  r++;
  layout_options->addWidget(new QLabel(tr("Target X (m):"), grp_options_), r, 0);
  layout_options->addWidget(spin_x_, r, 1);
  r++;
  layout_options->addWidget(new QLabel(tr("Target Y (m):"), grp_options_), r, 0);
  layout_options->addWidget(spin_y_, r, 1);
  r++;
  layout_options->addWidget(new QLabel(tr("Target Z (m):"), grp_options_), r, 0);
  layout_options->addWidget(spin_z_, r, 1);
  r++;
  layout_options->addWidget(new QLabel(tr("Target Yaw (°):"), grp_options_), r, 0);
  layout_options->addWidget(spin_yaw_, r, 1);

  layout_options->setRowStretch(r + 1, 1);
  layout_columns->addWidget(grp_options_, 1);

  tab_widget_->addTab(tab_planning, tr("Planning"));

  // ═══════════════════════════════════════════════════════════════════════════
  // Tab 2 & 3: Joints & Scene Objects
  // ═══════════════════════════════════════════════════════════════════════════
  auto * tab_joints = new QWidget(this);
  auto * tab_scene = new QWidget(this);
  tab_widget_->addTab(tab_joints, tr("Joints"));
  tab_widget_->addTab(tab_scene, tr("Scene Objects"));

  // Select Tab 1 ("Planning") by default
  tab_widget_->setCurrentIndex(1);

  root_layout->addWidget(tab_widget_);

  // Connect Signals & Slots
  connect(btn_plan_exec_, &QPushButton::clicked,
          this, &BirobotControlPanel::onPlanAndExecuteClicked);
  connect(btn_randomize_, &QPushButton::clicked,
          this, &BirobotControlPanel::onRandomizeClicked);
  connect(btn_apply_pose_, &QPushButton::clicked,
          this, &BirobotControlPanel::onApplyPoseClicked);
  connect(btn_stop_, &QPushButton::clicked,
          this, &BirobotControlPanel::onStopClicked);
  connect(btn_move_home_, &QPushButton::clicked,
          this, &BirobotControlPanel::onMoveHomeClicked);
  connect(combo_zone_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &BirobotControlPanel::onZoneChanged);
}

void BirobotControlPanel::applyMoveItStyle()
{
  setStyleSheet(
    "QWidget { "
    "  background-color: #24272b; "
    "  color: #e0e2e4; "
    "  font-family: 'Ubuntu', 'DejaVu Sans', sans-serif; "
    "  font-size: 11px; "
    "} "
    "QTabWidget::pane { "
    "  border: 1px solid #3c4043; "
    "  border-radius: 4px; "
    "  background-color: #24272b; "
    "  top: -1px; "
    "} "
    "QTabBar::tab { "
    "  background: #1c1e22; "
    "  color: #9aa0a6; "
    "  border: 1px solid #32363a; "
    "  border-bottom: none; "
    "  border-top-left-radius: 3px; "
    "  border-top-right-radius: 3px; "
    "  min-width: 60px; "
    "  padding: 4px 10px; "
    "  margin-right: 2px; "
    "} "
    "QTabBar::tab:selected { "
    "  background: #24272b; "
    "  color: #ffffff; "
    "  border: 1px solid #3c4043; "
    "  border-bottom: 1px solid #24272b; "
    "} "
    "QTabBar::tab:hover:!selected { "
    "  background: #202327; "
    "  color: #e0e2e4; "
    "} "
    "QGroupBox { "
    "  border: 1px solid #3c4043; "
    "  border-radius: 4px; "
    "  margin-top: 10px; "
    "  padding-top: 8px; "
    "  background-color: #24272b; "
    "} "
    "QGroupBox::title { "
    "  subcontrol-origin: margin; "
    "  subcontrol-position: top center; "
    "  padding: 0 6px; "
    "  color: #e0e2e4; "
    "  background-color: #24272b; "
    "} "
    "QPushButton { "
    "  background-color: #2b2f34; "
    "  border: 1px solid #4a4f54; "
    "  border-radius: 3px; "
    "  color: #e0e2e4; "
    "  padding: 4px 8px; "
    "} "
    "QPushButton:hover { "
    "  background-color: #383e45; "
    "  border-color: #5c636a; "
    "} "
    "QPushButton:pressed { "
    "  background-color: #1e2125; "
    "  border-color: #383c40; "
    "} "
    "QPushButton:disabled { "
    "  background-color: #222528; "
    "  border-color: #303336; "
    "  color: #555a60; "
    "} "
    "QComboBox { "
    "  background-color: #25282c; "
    "  border: 1px solid #4a4f54; "
    "  border-radius: 3px; "
    "  color: #e0e2e4; "
    "  padding: 2px 6px; "
    "} "
    "QComboBox:hover { "
    "  border-color: #5c636a; "
    "} "
    "QComboBox::drop-down { "
    "  subcontrol-origin: padding; "
    "  subcontrol-position: top right; "
    "  width: 18px; "
    "  border-left: 1px solid #3c4043; "
    "} "
    "QComboBox::down-arrow { "
    "  border-left: 4px solid transparent; "
    "  border-right: 4px solid transparent; "
    "  border-top: 5px solid #a0a4a8; "
    "  width: 0; "
    "  height: 0; "
    "} "
    "QComboBox QAbstractItemView { "
    "  background-color: #25282c; "
    "  color: #e0e2e4; "
    "  border: 1px solid #4a4f54; "
    "  selection-background-color: #3a424a; "
    "  selection-color: #ffffff; "
    "} "
    "QDoubleSpinBox, QSpinBox { "
    "  background-color: #25282c; "
    "  border: 1px solid #4a4f54; "
    "  border-radius: 3px; "
    "  color: #e0e2e4; "
    "  padding: 2px 4px; "
    "} "
    "QDoubleSpinBox:hover, QSpinBox:hover { "
    "  border-color: #5c636a; "
    "} "
    "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button, "
    "QSpinBox::up-button, QSpinBox::down-button { "
    "  width: 14px; "
    "  background-color: #2b2f34; "
    "  border-left: 1px solid #3c4043; "
    "} "
    "QDoubleSpinBox::up-arrow, QSpinBox::up-arrow { "
    "  border-left: 3px solid transparent; "
    "  border-right: 3px solid transparent; "
    "  border-bottom: 4px solid #a0a4a8; "
    "  width: 0; "
    "  height: 0; "
    "} "
    "QDoubleSpinBox::down-arrow, QSpinBox::down-arrow { "
    "  border-left: 3px solid transparent; "
    "  border-right: 3px solid transparent; "
    "  border-top: 4px solid #a0a4a8; "
    "  width: 0; "
    "  height: 0; "
    "} "
  );
}

void BirobotControlPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  if (!node_) return;

  client_randomize_ = node_->create_client<birobot_interfaces::srv::RandomizeObject>("/birobot/randomize_object");
  client_trigger_handover_ = node_->create_client<std_srvs::srv::Trigger>("/birobot/trigger_handover");
  client_reset_mission_ = node_->create_client<std_srvs::srv::Trigger>("/birobot/reset_mission");

  sub_mission_status_ = node_->create_subscription<std_msgs::msg::String>(
    "/birobot/mission_status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      Q_EMIT missionStatusReceived(QString::fromStdString(msg->data));
    });
}

void BirobotControlPanel::onZoneChanged(int index)
{
  QString code = combo_zone_->itemData(index).toString();
  if (code == "other_side") {
    spin_x_->setValue(-0.950);
    spin_y_->setValue(0.000);
    spin_z_->setValue(0.100);
  } else if (code == "front") {
    spin_x_->setValue(-0.350);
    spin_y_->setValue(0.000);
    spin_z_->setValue(0.150);
  } else if (code == "all") {
    spin_x_->setValue(-0.600);
    spin_y_->setValue(0.000);
    spin_z_->setValue(0.100);
  }
}

std::string BirobotControlPanel::getSelectedZoneCode() const
{
  return combo_zone_->currentData().toString().toStdString();
}

void BirobotControlPanel::onRandomizeClicked()
{
  if (!client_randomize_) return;

  auto req = std::make_shared<birobot_interfaces::srv::RandomizeObject::Request>();
  req->zone = getSelectedZoneCode();
  req->custom_pose = false;

  btn_randomize_->setEnabled(false);

  client_randomize_->async_send_request(
    req,
    [this](rclcpp::Client<birobot_interfaces::srv::RandomizeObject>::SharedFuture future) {
      try {
        auto res = future.get();
        Q_EMIT objectPoseUpdated(res->x, res->y, res->z, res->yaw,
                                 QString::fromStdString(res->location_desc));
      } catch (const std::exception & e) {
        Q_EMIT missionStatusReceived(QString("Randomize failed: %1").arg(e.what()));
      }
    });
}

void BirobotControlPanel::onApplyPoseClicked()
{
  if (!client_randomize_) return;

  auto req = std::make_shared<birobot_interfaces::srv::RandomizeObject::Request>();
  req->zone = "custom";
  req->custom_pose = true;
  req->custom_x = spin_x_->value();
  req->custom_y = spin_y_->value();
  req->custom_z = spin_z_->value();
  req->custom_yaw = spin_yaw_->value() * 0.0174532925;

  btn_apply_pose_->setEnabled(false);

  client_randomize_->async_send_request(
    req,
    [this](rclcpp::Client<birobot_interfaces::srv::RandomizeObject>::SharedFuture future) {
      try {
        auto res = future.get();
        Q_EMIT objectPoseUpdated(res->x, res->y, res->z, res->yaw,
                                 QString::fromStdString(res->location_desc));
      } catch (const std::exception & e) {
        Q_EMIT missionStatusReceived(QString("Apply pose failed: %1").arg(e.what()));
      }
    });
}

void BirobotControlPanel::updateObjectPoseFromSignal(double x, double y, double z, double yaw, const QString & desc)
{
  btn_randomize_->setEnabled(true);
  btn_apply_pose_->setEnabled(true);

  spin_x_->setValue(x);
  spin_y_->setValue(y);
  spin_z_->setValue(z);
  spin_yaw_->setValue(yaw * 57.2957795);

  QString state_str = QString("<object: %1>").arg(desc);
  if (combo_state_->findText(state_str) == -1) {
    combo_state_->addItem(state_str);
  }
  combo_state_->setCurrentText(state_str);
}

void BirobotControlPanel::onPlanAndExecuteClicked()
{
  if (!client_trigger_handover_) return;

  btn_plan_exec_->setEnabled(false);
  btn_randomize_->setEnabled(false);
  btn_apply_pose_->setEnabled(false);
  btn_move_home_->setEnabled(false);
  btn_stop_->setEnabled(true);

  QString running_str = tr("<current: RUNNING>");
  if (combo_state_->findText(running_str) == -1) {
    combo_state_->addItem(running_str);
  }
  combo_state_->setCurrentText(running_str);

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  client_trigger_handover_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto res = future.get();
        if (!res->success) {
          Q_EMIT missionStatusReceived(QString("FAILED: %1").arg(QString::fromStdString(res->message)));
        }
      } catch (const std::exception & e) {
        Q_EMIT missionStatusReceived(QString("FAILED: %1").arg(e.what()));
      }
    });
}

void BirobotControlPanel::onStopClicked()
{
  if (!client_reset_mission_) return;

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  client_reset_mission_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto res = future.get();
        Q_EMIT missionStatusReceived(QString::fromStdString(res->message));
      } catch (const std::exception & e) {
        Q_EMIT missionStatusReceived(QString("Stop failed: %1").arg(e.what()));
      }
    });
}

void BirobotControlPanel::onMoveHomeClicked()
{
  onStopClicked();
}

void BirobotControlPanel::updateStatusFromSignal(const QString & status_text)
{
  QString formatted = QString("<%1>").arg(status_text);
  if (combo_state_->findText(formatted) == -1) {
    combo_state_->addItem(formatted);
  }
  combo_state_->setCurrentText(formatted);

  if (status_text.startsWith("IDLE") || status_text.startsWith("SUCCESS") ||
      status_text.startsWith("FAILED") || status_text.startsWith("ABORTED"))
  {
    btn_plan_exec_->setEnabled(true);
    btn_randomize_->setEnabled(true);
    btn_apply_pose_->setEnabled(true);
    btn_move_home_->setEnabled(true);
    btn_stop_->setEnabled(false);
  } else if (status_text.startsWith("RUNNING")) {
    btn_plan_exec_->setEnabled(false);
    btn_randomize_->setEnabled(false);
    btn_apply_pose_->setEnabled(false);
    btn_move_home_->setEnabled(false);
    btn_stop_->setEnabled(true);
  }
}

void BirobotControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ZoneIndex", combo_zone_->currentIndex());
  config.mapSetValue("CustomX", spin_x_->value());
  config.mapSetValue("CustomY", spin_y_->value());
  config.mapSetValue("CustomZ", spin_z_->value());
  config.mapSetValue("CustomYaw", spin_yaw_->value());
  config.mapSetValue("PlanningTime", spin_time_->value());
  config.mapSetValue("VelocityScaling", spin_velocity_->value());
}

void BirobotControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  int zone_idx = 0;
  if (config.mapGetInt("ZoneIndex", &zone_idx)) {
    combo_zone_->setCurrentIndex(zone_idx);
  }
  float val = 0.0f;
  if (config.mapGetFloat("CustomX", &val)) spin_x_->setValue(val);
  if (config.mapGetFloat("CustomY", &val)) spin_y_->setValue(val);
  if (config.mapGetFloat("CustomZ", &val)) spin_z_->setValue(val);
  if (config.mapGetFloat("CustomYaw", &val)) spin_yaw_->setValue(val);
  if (config.mapGetFloat("PlanningTime", &val)) spin_time_->setValue(val);
  if (config.mapGetFloat("VelocityScaling", &val)) spin_velocity_->setValue(val);
}

}  // namespace birobot_rviz_plugins

PLUGINLIB_EXPORT_CLASS(birobot_rviz_plugins::BirobotControlPanel, rviz_common::Panel)
