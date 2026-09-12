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


def set_gazebo_pose(name, x, y, z, yaw):
    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
    req_str = (
        f'name: "{name}", '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
        f'orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}'
    )
    cmd = [
        'gz', 'service',
        '-s', '/world/empty/set_pose',
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
        '--z', type=float, default=0.08,
        help='Target Z coordinate in meters (default: 0.08)'
    )
    parser.add_argument(
        '--yaw', type=float, default=None,
        help='Target Yaw orientation in radians'
    )
    parser.add_argument(
        '--random', action='store_true', default=False,
        help='Force random repositioning within dual-arm safe bounds'
    )
    args = parser.parse_args()

    # If neither x nor y specified, or --random set, pick random safe bounds
    if args.random or (args.x is None and args.y is None):
        target_x = round(random.uniform(-0.05, 0.15), 3)
        target_y = round(random.uniform(-0.18, 0.18), 3)
        target_z = 0.08
        target_yaw = round(random.uniform(-1.5708, 1.5708), 3)
    else:
        target_x = args.x if args.x is not None else 0.10
        target_y = args.y if args.y is not None else 0.05
        target_z = args.z
        target_yaw = args.yaw if args.yaw is not None else 0.40

    success = set_gazebo_pose(args.name, target_x, target_y, target_z, target_yaw)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
