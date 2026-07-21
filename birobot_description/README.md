# birobot_description

`birobot_description` is the Layer 1 Hardware Abstraction (Geometry) package for the **Birobot** dual-arm collaborative system. It provides a unified URDF (via xacro) that defines two Universal Robots UR10e arms mounted in a face-to-face opposing layout on a shared workcell table, rooted at a single `world` coordinate frame.

---

## Workspace Setup

### Prerequisites

Ensure ROS 2 (Humble or Jazzy) is installed and sourced, along with standard ROS description dependencies:

```bash
ros2 pkg list | grep -E "xacro|robot_state_publisher|joint_state_publisher_gui|rviz2|tf2_tools"
```

### Build and Install

```bash
# Clone repository and initialize git submodules
git clone <repository-url> Birobot
cd Birobot
git submodule update --init --recursive

# Build package with colcon
export TMPDIR=/home/mox/tmp # Ensure sufficient temp space if root partition is constrained
colcon build --packages-select birobot_description ur_description
source install/setup.bash
```

---

## Usage

### Quick Launch

To launch the dual-arm workcell visualization with RViz2 and the Joint State Publisher GUI:

```bash
ros2 launch birobot_description display.launch.py start_rviz:=true
```

### Parameter Overrides

Mount positions and orientations can be overriden at launch time:

```bash
# Shift Arm 1 mount X position to -0.7 m
ros2 launch birobot_description display.launch.py arm1_x:=-0.700 start_rviz:=true
```

---

## Transform Reference

The mount positions and derived inter-arm transform ground truth are structured as follows:

| Transform | Parent Link | Child Link | Translation XYZ (m) | Rotation RPY (rad) |
|-----------|-------------|------------|---------------------|-------------------|
| $^{W}T_{B1}$ | `workcell_base_link` | `arm1_base_link` | `[-0.600, 0.000, 0.000]` | `[0.000, 0.000, 0.000]` |
| $^{W}T_{B2}$ | `workcell_base_link` | `arm2_base_link` | `[+0.600, 0.000, 0.000]` | `[0.000, 0.000, 3.14159]` |
| $^{B1}T_{B2}$ (Derived) | `arm1_base_link` | `arm2_base_link` | `[1.200, 0.000, 0.000]` | `[0.000, 0.000, 3.14159]` |

### Expected `tf2_echo` Output

```text
At time <T>
- Translation: [1.200, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 1.000, 0.000]
             in RPY (radian) [0.000, -0.000, 3.14159]
             in RPY (degree) [0.000, -0.000, 180.000]
```

---

## Coordinate Frame Convention

This package adheres strictly to **REP-103**:
- **X-axis**: Forward / Right (arm separation along X axis)
- **Y-axis**: Left
- **Z-axis**: Up (perpendicular to table top)

**World Frame Semantics**: The `world` origin is positioned at the geometric centre of the top surface of the workcell table (`workcell_base_link`) at $Z = 0.000\text{ m}$.

---

## Vendor Attribution

The UR10e kinematic chain and mesh descriptions are provided via an official Universal Robots description submodule:

- **Upstream Repository**: [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- **Branch**: `humble`
- **Pinned Commit SHA**: `e0c6bab1682f53e3fb8aaae7eb9bf52f21b44171`
- **Release Version**: `2.12.0`
- **Submodule Path**: `birobot_description/vendor/ur_description`
- **Licence**: BSD-3-Clause

---

## Validation Status

| Scenario | Goal | Command | Status |
|----------|------|---------|--------|
| S1: Dual-Arm Launch | RViz2 dual arm rendering | `ros2 launch birobot_description display.launch.py` | PASS |
| S2: TF Completeness | 18 connected TF frames | `ros2 run tf2_tools view_frames` | PASS |
| S3: Transform Accuracy | $^{B1}T_{B2} = [1.2, 0, 0] / [0, 0, \pi]$ | `ros2 run tf2_ros tf2_echo arm1_base_link arm2_base_link` | PASS |
| S4: Clean Build | Zero build errors/warnings | `colcon build --packages-select birobot_description` | PASS |
| S5: Asset Isolation | UR10e-only mesh references | `grep -r "meshes/" urdf/ \| grep -v "ur10e"` | PASS |
| S6: Parameter Override | Override `arm1_x` dynamically | `ros2 launch ... arm1_x:=-0.7` | PASS |
| S7: Spacing Guard | Abort on spacing < 0.200 m | `ros2 launch ... arm1_x:=-0.05 arm2_x:=0.05` | PASS |
