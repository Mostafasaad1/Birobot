#!/usr/bin/env python3
"""
randomize_object.py — Reposition irregular_object_1 live in Gazebo Sim.

Can reposition the object to a random safe location within the collaborative
dual-arm & camera workspace, or to explicit coordinates passed via CLI.

Workspace bounds:
  X:   [-0.05, 0.15] m
  Y:   [-0.18, 0.18] m
  Z:   0.080 m
  Yaw: [-1.57, 1.57] rad
"""

import argparse
import math
import random
import subprocess
import sys


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def find_set_pose_service():
    """Detect available /world/<name>/set_pose service from running Gazebo."""
    try:
        res = subprocess.run(['gz', 'service', '-l'], capture_output=True, text=True, timeout=2)
        if res.returncode == 0:
            for line in res.stdout.splitlines():
                line = line.strip()
                if line.startswith('/world/') and line.endswith('/set_pose'):
                    return line
    except Exception:
        pass
    return None


def set_gazebo_pose(name, x, y, z, yaw):
    service_name = find_set_pose_service()
    if not service_name:
        # Check if Gazebo is running at all
        print("\n[NOTE] Gazebo Sim does not appear to be currently running.")
        print("Note that 'randomize_object.py' repositions models LIVE inside an active Gazebo simulation.")
        print("To launch the simulation (which automatically spawns the object past the table by default):")
        print("  ros2 launch birobot_moveit_config gazebo.launch.py")
        print("or")
        print("  ros2 launch birobot_manipulation autonomous_system.launch.py\n")
        service_name = '/world/empty/set_pose'

    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
    req_str = (
        f'name: "{name}", '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
        f'orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}'
    )
    cmd = [
        'gz', 'service',
        '-s', service_name,
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', req_str
    ]
    print(f"Executing: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0 and "data: true" in result.stdout:
        print(f"Successfully relocated '{name}' to:")
        print(f"  X   = {x:.3f} m")
        print(f"  Y   = {y:.3f} m")
        print(f"  Z   = {z:.3f} m")
        print(f"  Yaw = {yaw:.3f} rad ({math.degrees(yaw):.1f} deg)")
        return True
    else:
        print(f"Failed to relocate '{name}'. Gazebo output:")
        print(result.stdout)
        print(result.stderr)
        if "Service call timed out" in result.stdout or "Service call timed out" in result.stderr:
            print("\n[HINT] 'Service call timed out' happens when Gazebo Sim is not running.")
            print("Please keep Gazebo running in Terminal 1, then run this command in Terminal 2.\n")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Reposition irregular_object_1 live in Gazebo Sim."
    )
    parser.add_argument(
        '--name', default='irregular_object_1',
        help='Name of model to reposition (default: irregular_object_1)'
    )
    parser.add_argument(
        '--x', type=float, default=None,
        help='Target X coordinate in meters'
    )
    parser.add_argument(
        '--y', type=float, default=None,
        help='Target Y coordinate in meters'
    )
    parser.add_argument(
        '--z', type=float, default=None,
        help='Target Z coordinate in meters (default: auto-detected, 0.10 past table or 0.15 on table)'
    )
    parser.add_argument(
        '--yaw', type=float, default=None,
        help='Target Yaw orientation in radians'
    )
    parser.add_argument(
        '--random', action='store_true', default=False,
        help='Force random repositioning within dual-arm safe bounds'
    )
    parser.add_argument(
        '--zone', default='other_side', choices=['other_side', 'front', 'all'],
        help='Spawn zone around Arm 1: other_side (past table border X in [-1.08, -0.90]), front (on table X in [-0.48, -0.15]), or all (default: other_side)'
    )
    args = parser.parse_args()

    # Determine if random repositioning is requested:
    force_random = getattr(args, 'random', False)
    if force_random or (args.x is None and args.y is None):
        while True:
            if args.zone in ('other_side', 'otherside', 'rear', 'back'):
                # Past table border behind Arm 1:
                # Table ends at X = -0.800. Object footprint radius is ~0.085m.
                # Center X in [-1.080, -0.900] guarantees object closest edge is <= -0.815m (100% off table).
                x_val = random.uniform(-1.080, -0.900)
                y_val = random.uniform(-0.250, 0.250)
                dist_base = math.hypot(x_val - (-0.60), y_val)
                if 0.24 <= dist_base <= 0.52:
                    break
            elif args.zone in ('front', 'infront'):
                x_val = random.uniform(-0.480, -0.150)
                y_val = random.uniform(-0.280, 0.280)
                dist_base = math.hypot(x_val - (-0.60), y_val)
                if 0.24 <= dist_base <= 0.52:
                    break
            else:  # all
                subzone = 'other_side' if random.random() < 0.70 else 'front'
                if subzone == 'other_side':
                    x_val = random.uniform(-1.080, -0.900)
                    y_val = random.uniform(-0.250, 0.250)
                else:
                    x_val = random.uniform(-0.480, -0.150)
                    y_val = random.uniform(-0.280, 0.280)
                dist_base = math.hypot(x_val - (-0.60), y_val)
                if 0.24 <= dist_base <= 0.52:
                    break

        target_x = round(x_val, 3)
        target_y = round(y_val, 3)
        # Table top is at Z=0.05, ground is at Z=0.00. Object height is 0.20m.
        is_on_table = (-0.80 <= target_x <= 0.80) and (-0.40 <= target_y <= 0.40)
        target_z = 0.15 if is_on_table else 0.10
        target_yaw = round(random.uniform(-1.5708, 1.5708), 3)
    else:
        target_x = args.x if args.x is not None else -0.95
        target_y = args.y if args.y is not None else 0.00
        is_on_table = (-0.80 <= target_x <= 0.80) and (-0.40 <= target_y <= 0.40)
        target_z = args.z if args.z is not None else (0.15 if is_on_table else 0.10)
        target_yaw = args.yaw if args.yaw is not None else 0.00

    location_desc = "ON TABLE" if (-0.80 <= target_x <= 0.80 and -0.40 <= target_y <= 0.40) else "GROUND (PAST TABLE BORDER)"
    print(f"Target location selected: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) [{location_desc}]")

    success = set_gazebo_pose(args.name, target_x, target_y, target_z, target_yaw)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
