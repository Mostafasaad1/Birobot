# Birobot - Autonomous Robotic Manipulation System

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## Overview

**Birobot** is a complete ROS 2 robotic manipulation system integrating a Universal Robots UR10e manipulator with advanced perception, motion planning, and autonomous task execution capabilities. The system demonstrates industrial-grade pick-and-place operations using state-of-the-art robotics frameworks.

### Key Features

- **🤖 Full UR10e Integration**: Complete URDF model with gripper and sensor mounting
- **👁️ 3D Vision Perception**: RGB-D camera integration with PCL-based object detection
- **🎯 Motion Planning**: MoveIt 2 integration with collision avoidance
- **🔄 Task Orchestration**: MoveIt Task Constructor (MTC) for complex manipulation tasks
- **🌳 Behavior Trees**: BehaviorTree.CPP for high-level task coordination
- **♻️ Lifecycle Management**: Full lifecycle node architecture for robust state management
- **🔬 Comprehensive Testing**: Unit and integration tests for all components

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Birobot System Architecture                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────┐         ┌──────────────────┐             │
│  │  BT Coordinator  │◄────────┤  Pick & Place    │             │
│  │     (BT.CPP)     │         │   Action Server  │             │
│  └────────┬─────────┘         └────────┬─────────┘             │
│           │                             │                        │
│           │ Orchestration               │ MTC Pipeline           │
│           ▼                             ▼                        │
│  ┌─────────────────────────────────────────────────┐            │
│  │         MoveIt Task Constructor (MTC)           │            │
│  │  - Pick Stage      - Place Stage                │            │
│  │  - Approach/Retreat - Connect/Merge             │            │
│  └─────────────┬───────────────────────────────────┘            │
│                │                                                 │
│                │ Motion Plans                                    │
│                ▼                                                 │
│  ┌──────────────────────────────────────────────────┐           │
│  │              MoveIt 2 Framework                   │           │
│  │  - OMPL Motion Planning                          │           │
│  │  - Collision Detection (FCL)                     │           │
│  │  - Kinematics (KDL)                              │           │
│  └─────────────┬────────────────────────────────────┘           │
│                │                                                 │
│  ┌─────────────┴────────────────────────────────────┐           │
│  │           Perception Pipeline                     │           │
│  │  - Point Cloud Processing (PCL)                  │           │
│  │  - RANSAC Plane Detection                        │           │
│  │  - PCA Pose Estimation                           │           │
│  └─────────────┬────────────────────────────────────┘           │
│                │                                                 │
│                │ Object Poses                                    │
│                ▼                                                 │
│  ┌──────────────────────────────────────────────────┐           │
│  │            Robot Hardware Layer                   │           │
│  │  - UR10e Robot (ros2_control)                    │           │
│  │  - 2F Gripper (Joint Commands)                   │           │
│  │  - RGB-D Camera (Sensor Data)                    │           │
│  └──────────────────────────────────────────────────┘           │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Project Structure

```
Birobot/
├── src/
│   ├── birobot_description/          # Robot model & visualization
│   │   ├── urdf/                     # URDF/xacro robot definitions
│   │   │   ├── birobot.urdf.xacro   # Main robot assembly
│   │   │   └── grippers/             # Gripper models
│   │   ├── config/                   # Controller & initial pose configs
│   │   └── launch/                   # Visualization launch files
│   │
│   ├── birobot_moveit_config/        # MoveIt 2 configuration
│   │   ├── config/                   # Planning, kinematics, SRDF
│   │   │   ├── birobot.srdf         # Semantic robot description
│   │   │   ├── kinematics.yaml      # IK solver configuration
│   │   │   ├── ompl_planning.yaml   # Motion planning parameters
│   │   │   └── joint_limits.yaml    # Joint constraints
│   │   └── launch/                   # MoveIt launch files
│   │
│   ├── birobot_perception/           # Vision & perception system
│   │   ├── src/
│   │   │   ├── perception_node.cpp           # Main perception node
│   │   │   └── irregular_object_pose_estimator.cpp  # PCA estimation
│   │   ├── config/
│   │   │   └── perception_params.yaml        # Perception parameters
│   │   └── test/                     # Perception unit tests
│   │
│   ├── birobot_manipulation/         # Task execution & planning
│   │   ├── src/
│   │   │   ├── mtc_pick_place_node.cpp       # MTC pick-and-place
│   │   │   ├── birobot_bt_coordinator_node.cpp  # BT coordinator
│   │   │   └── bt_nodes/             # Custom BehaviorTree nodes
│   │   ├── config/
│   │   │   └── bt_trees/             # Behavior tree definitions
│   │   ├── launch/                   # Manipulation launch files
│   │   └── test/                     # Manipulation tests
│   │
│   ├── birobot_interfaces/           # Custom ROS 2 interfaces
│   │   └── action/
│   │       └── PickAndPlace.action   # Pick-and-place action definition
│   │
│   ├── ur_description/               # UR10e robot description (submodule)
│   ├── moveit_task_constructor/      # MTC framework (submodule)
│   └── py_binding_tools/             # Python bindings (submodule)
│
├── README.md                         # This file
└── skills-lock.json                  # Dependency lock file
```

---

## Prerequisites

### System Requirements
- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2**: Jazzy Jalisco
- **RAM**: Minimum 8GB (16GB recommended)
- **CPU**: Multi-core processor (4+ cores recommended)

### Required Dependencies

```bash
# ROS 2 Jazzy (Full Desktop)
sudo apt update
sudo apt install ros-jazzy-desktop-full

# MoveIt 2
sudo apt install ros-jazzy-moveit

# Robot Control & Simulation
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gazebo-ros2-control \
  ros-jazzy-xacro

# Point Cloud Library (PCL)
sudo apt install \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  libpcl-dev

# BehaviorTree.CPP
sudo apt install ros-jazzy-behaviortree-cpp-v3

# Additional Tools
sudo apt install \
  ros-jazzy-tf2-tools \
  ros-jazzy-rqt-tf-tree \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  git
```

---

## Installation

### 1. Clone the Repository

```bash
# Create workspace
mkdir -p ~/birobot_ws/src
cd ~/birobot_ws/src

# Clone with submodules
git clone --recursive https://github.com/yourusername/Birobot.git
cd Birobot

# If already cloned without --recursive
git submodule update --init --recursive
```

### 2. Install Dependencies

```bash
cd ~/birobot_ws

# Use rosdep to install all dependencies
sudo rosdep init  # If not already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
cd ~/birobot_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 4. Verify Installation

```bash
# Check if packages are available
ros2 pkg list | grep birobot

# Expected output:
# birobot_description
# birobot_interfaces
# birobot_manipulation
# birobot_moveit_config
# birobot_perception
```

---

## Usage

### 1. Visualize Robot in RViz

```bash
# Terminal 1: Launch robot visualization
ros2 launch birobot_description display.launch.py

# You should see the UR10e robot with gripper in RViz
# Use the Joint State Publisher GUI to move joints
```

### 2. Launch MoveIt Planning

```bash
# Terminal 1: Launch MoveIt with simulation
ros2 launch birobot_moveit_config demo.launch.py

# Interact with the robot in RViz:
# - Use the Motion Planning panel to plan trajectories
# - Drag the interactive marker to set goal poses
# - Click "Plan" and "Execute" to move the robot
```

### 3. Run Perception Pipeline

```bash
# Terminal 1: Launch perception with mock camera
ros2 launch birobot_perception perception_pipeline.launch.py

# Terminal 2: Publish test point cloud (example)
ros2 topic pub /camera/depth/color/points sensor_msgs/msg/PointCloud2 ...

# Terminal 3: Monitor detected objects
ros2 topic echo /perception/detected_objects
```

### 4. Execute Pick-and-Place Tasks

```bash
# Terminal 1: Launch complete autonomous system
ros2 launch birobot_manipulation autonomous_system.launch.py

# This launches:
# - MoveIt motion planning
# - Perception pipeline
# - MTC pick-and-place node
# - BehaviorTree coordinator

# Terminal 2: Trigger a pick-and-place action
ros2 action send_goal /pick_and_place birobot_interfaces/action/PickAndPlace \
  "{pick_pose: {position: {x: 0.5, y: 0.0, z: 0.1}, orientation: {w: 1.0}}, \
    place_pose: {position: {x: 0.3, y: 0.3, z: 0.2}, orientation: {w: 1.0}}}"
```

### 5. Simulation with Gazebo

```bash
# Launch robot in Gazebo with MoveIt
ros2 launch birobot_moveit_config gazebo.launch.py

# Control the robot through MoveIt in RViz or programmatically
```

---

## Running Tests

```bash
cd ~/birobot_ws

# Build with tests enabled
colcon build --symlink-install

# Run all tests
colcon test

# View test results
colcon test-result --verbose

# Run specific package tests
colcon test --packages-select birobot_perception
colcon test --packages-select birobot_manipulation
```

### Test Coverage

- **Perception Tests**:
  - `test_pointcloud_subscriber`: Point cloud data reception
  - `test_ransac_filtering`: Table plane detection
  - `test_pca_pose_estimation`: Object orientation estimation
  - `test_end_to_end_perception`: Full pipeline integration

- **Manipulation Tests**:
  - `test_mtc_pipeline`: MTC stage execution
  - `test_bt_nodes`: BehaviorTree node functionality

---

## Configuration

### Perception Parameters

Edit `src/birobot_perception/config/perception_params.yaml`:

```yaml
perception_node:
  ros__parameters:
    # Point cloud processing
    voxel_leaf_size: 0.005        # Downsample resolution (meters)
    ransac_distance_threshold: 0.01  # Plane detection tolerance
    ransac_max_iterations: 1000    # RANSAC iterations
    
    # Object detection
    cluster_tolerance: 0.02        # Euclidean clustering distance
    min_cluster_size: 100          # Minimum points per object
    max_cluster_size: 25000        # Maximum points per object
```

### Motion Planning Parameters

Edit `src/birobot_moveit_config/config/ompl_planning.yaml`:

```yaml
planning:
  planner_configs:
    RRTConnect:
      type: geometric::RRTConnect
      range: 0.0
    RRTstar:
      type: geometric::RRTstar
      range: 0.0
      goal_bias: 0.05
```

### Controller Configuration

Edit `src/birobot_description/config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    gripper_controller:
      type: position_controllers/JointGroupPositionController
```

---

## Package Descriptions

### birobot_description
Robot model package containing URDF/xacro definitions, meshes, and visualization tools.
- **Main File**: `urdf/birobot.urdf.xacro`
- **Includes**: UR10e robot, 2F gripper, sensor mounts

### birobot_moveit_config
MoveIt 2 configuration for motion planning and control.
- **SRDF**: Semantic robot description with planning groups
- **Kinematics**: KDL solver configuration
- **Planning**: OMPL planner settings

### birobot_perception
3D vision and object detection system using PCL.
- **Node**: `perception_node` (Lifecycle)
- **Features**: RANSAC plane removal, PCA pose estimation, clustering
- **Outputs**: Detected object poses in TF frame

### birobot_manipulation
High-level task planning and execution.
- **MTC Node**: Pick-and-place pipeline using MoveIt Task Constructor
- **BT Coordinator**: BehaviorTree-based task orchestration
- **Action Server**: ROS 2 action interface for task commands

### birobot_interfaces
Custom ROS 2 message, service, and action definitions.
- **Actions**: `PickAndPlace.action`

---

## Development

### Adding New Behavior Tree Nodes

1. Create node class in `src/birobot_manipulation/src/bt_nodes/`
2. Inherit from `BT::SyncActionNode` or `BT::AsyncActionNode`
3. Register in BT factory in `birobot_bt_coordinator_node.cpp`
4. Define in XML: `config/bt_trees/your_tree.xml`

### Creating New MTC Stages

```cpp
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>

using namespace moveit::task_constructor;

auto task = std::make_unique<Task>();
task->stages()->setName("custom_task");

// Add current state
auto current_state = std::make_unique<stages::CurrentState>("current");
task->add(std::move(current_state));

// Add custom stages...
```

### Extending Perception

1. Modify `irregular_object_pose_estimator.cpp` for new object types
2. Add custom filtering in `perception_node.cpp`
3. Update parameters in `perception_params.yaml`
4. Write tests in `test/`

---

## Troubleshooting

### Issue: MoveIt fails to find planning plugin

```bash
# Solution: Check OMPL plugin is installed
sudo apt install ros-jazzy-moveit-planners-ompl

# Verify in SRDF that planning plugin is correctly referenced
```

### Issue: Perception node receives no point clouds

```bash
# Check if camera topics are publishing
ros2 topic list | grep camera
ros2 topic hz /camera/depth/color/points

# Verify topic remapping in launch file
```

### Issue: Gripper doesn't close

```bash
# Check controller status
ros2 control list_controllers

# Manually test gripper
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0]}"  # Open
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.8]}"  # Close
```

### Issue: TF transform errors

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link end_effector_link

# Verify static transforms in launch files
```

---

## Performance Optimization

### Motion Planning
- Adjust planning time in MoveIt: `allowed_planning_time: 5.0`
- Use faster planners for simple motions: `RRTConnect` vs `RRTstar`
- Reduce collision check resolution: Increase `distance_threshold`

### Perception
- Increase voxel leaf size for faster processing: `0.01` vs `0.005`
- Reduce RANSAC iterations for speed: `500` vs `1000`
- Limit point cloud region of interest in camera driver

### Memory Usage
- Limit point cloud size: Use PassThrough filter before processing
- Reduce RViz visualization: Disable trajectory display, reduce markers

---

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork** the repository
2. **Create** a feature branch: `git checkout -b feature/amazing-feature`
3. **Commit** changes: `git commit -m 'Add amazing feature'`
4. **Push** to branch: `git push origin feature/amazing-feature`
5. **Open** a Pull Request

### Code Style
- Follow [ROS 2 style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use `clang-format` for C++ code
- Use `black` for Python code
- Add unit tests for new features

---

## Roadmap

- [ ] **Real Hardware Integration**: Deploy on physical UR10e robot
- [ ] **Advanced Grasping**: Integrate GraspIt! or GPD for grasp planning
- [ ] **Deep Learning**: YOLOv8 for object detection
- [ ] **Force Control**: Add force-torque sensor integration
- [ ] **Multi-Robot**: Extend to multi-arm coordination
- [ ] **Web Interface**: Add web-based monitoring dashboard
- [ ] **Docker Support**: Containerized deployment

---

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- [MoveIt Task Constructor](https://moveit.github.io/moveit_task_constructor/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Point Cloud Library](https://pointclouds.org/)
- [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

---

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **MoveIt 2 Team**: For the excellent motion planning framework
- **PickNik Robotics**: For MoveIt Task Constructor
- **Universal Robots**: For UR robot descriptions
- **ROS 2 Community**: For continuous support and tools

---

## Contact

**Maintainer**: mox  
**Email**: mox@todo.todo  
**Project**: [https://github.com/Mostafasaad1/Birobot](https://github.com/Mostafasaad1/Birobot)

---

## Citation

If you use this project in your research, please cite:

```bibtex
@software{birobot2026,
  author = {Mox},
  title = {Birobot: Autonomous Robotic Manipulation System},
  year = {2026},
  publisher = {GitHub},
  url = {https://github.com/Mostafasaad1/Birobot}
}
```
