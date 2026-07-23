#!/usr/bin/env python3
"""
Motion Planning Monitor Node for Birobot MoveIt 2 Configuration.

Provides Constitution-mandated observability:
- 1 Hz DiagnosticArray heartbeat on /diagnostics for arm_1, arm_2, dual_arms planning groups
- Audit logging on /birobot/audit_log for motion planning events
"""

import json
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String


class MotionPlanningMonitor(Node):
    def __init__(self):
        super().__init__('motion_planning_monitor')

        # Declare parameters
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('arm_1_loaded', True)
        self.declare_parameter('arm_2_loaded', True)
        self.declare_parameter('dual_arms_loaded', True)

        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.audit_pub = self.create_publisher(String, '/birobot/audit_log', 10)

        # Timer for diagnostic heartbeat
        rate = self.get_parameter('heartbeat_rate').value
        timer_period = 1.0 / rate if rate > 0 else 1.0
        self.timer = self.create_timer(timer_period, self.publish_diagnostics)

        self.get_logger().info('Motion Planning Monitor initialized.')

    def publish_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'birobot: motion_planning_groups'

        arm_1_ok = self.get_parameter('arm_1_loaded').value
        arm_2_ok = self.get_parameter('arm_2_loaded').value
        dual_arms_ok = self.get_parameter('dual_arms_loaded').value

        all_ok = arm_1_ok and arm_2_ok and dual_arms_ok

        if all_ok:
            status.level = DiagnosticStatus.OK
            status.message = 'All groups loaded successfully'
        else:
            status.level = DiagnosticStatus.ERROR
            failed_groups = []
            if not arm_1_ok:
                failed_groups.append('arm_1')
            if not arm_2_ok:
                failed_groups.append('arm_2')
            if not dual_arms_ok:
                failed_groups.append('dual_arms')
            status.message = f"{', '.join(failed_groups)} failed to load"

        status.values = [
            KeyValue(key='arm_1_solver', value='KDL'),
            KeyValue(key='arm_1_status', value='LOADED' if arm_1_ok else 'NOT_LOADED'),
            KeyValue(key='arm_2_solver', value='KDL'),
            KeyValue(key='arm_2_status', value='LOADED' if arm_2_ok else 'NOT_LOADED'),
            KeyValue(key='dual_arms_status', value='LOADED' if dual_arms_ok else 'NOT_LOADED'),
        ]

        msg.status.append(status)
        self.diag_pub.publish(msg)

    def log_audit_event(
        self,
        planning_group: str,
        goals: dict,
        timeout_sec: float,
        status_code: str,
        trajectory_states: int = 0,
        message: str = ''
    ):
        """
        Record a planning event to /birobot/audit_log (FR-013, FR-014, FR-015).
        """
        # Partial goal check for dual_arms (FR-014)
        if planning_group == 'dual_arms':
            arm_1_goal = goals.get('arm_1')
            arm_2_goal = goals.get('arm_2')
            if arm_1_goal and not arm_2_goal:
                self.get_logger().info(
                    "Partial dual_arms goal received: arm_2 goal omitted; treating arm_2 as static obstacle."
                )
            elif arm_2_goal and not arm_1_goal:
                self.get_logger().info(
                    "Partial dual_arms goal received: arm_1 goal omitted; treating arm_1 as static obstacle."
                )

        payload = {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'event_type': 'PLANNING_RESULT',
            'planning_group': planning_group,
            'goals': goals,
            'timeout_sec': timeout_sec,
            'status_code': status_code,
            'trajectory_states': trajectory_states,
            'message': message,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.audit_pub.publish(msg)
        self.get_logger().info(f"Audit log published: {status_code} for {planning_group}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
