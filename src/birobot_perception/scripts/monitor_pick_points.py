#!/usr/bin/env python3
"""
monitor_pick_points.py — Live terminal CLI monitor for 3D perception solved pick points.

Displays real-time object coordinates, PCA orientation, distances to both
robot arms, reachability status, and collaborative workspace validation.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray


# ANSI Colors
C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_RED = "\033[91m"
C_GREEN = "\033[92m"
C_YELLOW = "\033[93m"
C_BLUE = "\033[94m"
C_MAGENTA = "\033[95m"
C_CYAN = "\033[96m"
C_WHITE = "\033[97m"
C_DIM = "\033[2m"
C_BG_DARK = "\033[40m"


def quat_to_yaw(qx, qy, qz, qw):
    """Compute yaw angle in radians from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PickPointMonitor(Node):
    def __init__(self):
        super().__init__('pick_point_monitor')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub_poses = self.create_subscription(
            PoseArray,
            '/birobot/perception/target_poses',
            self.pose_callback,
            qos,
        )

        # UR5 arm base coordinates in world frame
        self.arm1_base = (-0.600, 0.000, 0.000)
        self.arm2_base = (0.600, 0.000, 0.000)
        self.max_arm_reach = 0.850  # UR5 max nominal reach

        self.last_poses = []
        self.msg_count = 0
        self.last_update_time = time.time()

        self.get_logger().info("PickPointMonitor initialized. Listening to /birobot/perception/target_poses...")
        print(f"{C_BOLD}{C_CYAN}=== Birobot 3D Perception Pick-Point Live Monitor ==={C_RESET}")
        print(f"Subscribing to: {C_WHITE}/birobot/perception/target_poses{C_RESET}\n")

    def pose_callback(self, msg: PoseArray):
        self.msg_count += 1
        now_wall = time.strftime("%H:%M:%S")
        sim_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        num_objects = len(msg.poses)

        # Clear screen line or print formatted header
        out = []
        out.append(f"\n{C_BOLD}{C_CYAN}╔══════════════════════════════════════════════════════════════════════════════════╗{C_RESET}")
        out.append(f"{C_BOLD}{C_CYAN}║ BIROBOT PERCEPTION LIVE MONITOR — [{now_wall}] Frame: {msg.header.frame_id:<8} Msg #{self.msg_count:<5} ║{C_RESET}")
        out.append(f"{C_BOLD}{C_CYAN}╠══════════════════════════════════════════════════════════════════════════════════╣{C_RESET}")

        if num_objects == 0:
            out.append(f"{C_YELLOW}║  NO OBJECTS DETECTED (Waiting for pointcloud clusters / RANSAC filtering)...   ║{C_RESET}")
        else:
            for i, p in enumerate(msg.poses):
                x = p.position.x
                y = p.position.y
                z = p.position.z

                yaw = quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                yaw_deg = math.degrees(yaw)

                # Distance to arm bases
                dist_arm1 = math.hypot(x - self.arm1_base[0], y - self.arm1_base[1])
                dist_arm2 = math.hypot(x - self.arm2_base[0], y - self.arm2_base[1])

                arm1_ok = dist_arm1 <= self.max_arm_reach
                arm2_ok = dist_arm2 <= self.max_arm_reach
                collab_zone = arm1_ok and arm2_ok

                # Object color tag (Obj 1 = Red, Obj 2 = Blue)
                if i == 0:
                    obj_tag = f"{C_BOLD}{C_RED}Object 1 (Red Target){C_RESET}"
                else:
                    obj_tag = f"{C_BOLD}{C_BLUE}Object {i+1} (Obstacle/Secondary){C_RESET}"

                collab_str = (
                    f"{C_GREEN}{C_BOLD}COLLABORATIVE SAFE ZONE{C_RESET}"
                    if collab_zone else
                    f"{C_YELLOW}SINGLE-ARM REACH ONLY{C_RESET}"
                )

                arm1_status = f"{C_GREEN}REACHABLE ({dist_arm1:.3f}m){C_RESET}" if arm1_ok else f"{C_RED}OUT OF RANGE ({dist_arm1:.3f}m){C_RESET}"
                arm2_status = f"{C_GREEN}REACHABLE ({dist_arm2:.3f}m){C_RESET}" if arm2_ok else f"{C_RED}OUT OF RANGE ({dist_arm2:.3f}m){C_RESET}"

                # Delta from previous if available
                delta_str = ""
                if len(self.last_poses) > i:
                    lx, ly, lz = self.last_poses[i]
                    drift = math.sqrt((x - lx)**2 + (y - ly)**2 + (z - lz)**2)
                    if drift < 0.005:
                        delta_str = f" {C_DIM}(Static / Stable, drift: {drift*1000:.1f}mm){C_RESET}"
                    else:
                        delta_str = f" {C_MAGENTA}(Dynamic / Moved: {drift*1000:.1f}mm){C_RESET}"

                out.append(f"║ {obj_tag}{delta_str}")
                out.append(f"║   ► Centroid Pick Point:  X = {C_BOLD}{x:+.4f}{C_RESET} m,  Y = {C_BOLD}{y:+.4f}{C_RESET} m,  Z = {C_BOLD}{z:+.4f}{C_RESET} m")
                out.append(f"║   ► PCA Orientation Yaw:  {C_BOLD}{yaw_deg:+6.1f}°{C_RESET}  ({yaw:+.3f} rad)")
                out.append(f"║   ► Arm 1 (Left Base):    {arm1_status}")
                out.append(f"║   ► Arm 2 (Right Base):   {arm2_status}")
                out.append(f"║   ► Workspace Zone:       {collab_str}")
                if i < num_objects - 1:
                    out.append(f"{C_CYAN}╟──────────────────────────────────────────────────────────────────────────────────╢{C_RESET}")

        out.append(f"{C_BOLD}{C_CYAN}╚══════════════════════════════════════════════════════════════════════════════════╝{C_RESET}")
        print("\n".join(out), flush=True)

        self.last_poses = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(
        description="Live terminal CLI monitor for 3D perception solved pick points."
    )
    parser.add_argument(
        '--topic', default='/birobot/perception/target_poses',
        help='Topic name for target poses (default: /birobot/perception/target_poses)'
    )
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = PickPointMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{C_YELLOW}PickPointMonitor terminated by user.{C_RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
