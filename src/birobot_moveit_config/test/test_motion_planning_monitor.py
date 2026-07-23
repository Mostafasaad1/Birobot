import json
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from birobot_moveit_config.motion_planning_monitor import MotionPlanningMonitor


def test_motion_planning_monitor_diagnostics():
    rclpy.init()
    try:
        node = MotionPlanningMonitor()

        received_diags = []
        received_audits = []

        sub_diag = node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            lambda msg: received_diags.append(msg),
            10
        )
        sub_audit = node.create_subscription(
            String,
            '/birobot/audit_log',
            lambda msg: received_audits.append(msg),
            10
        )

        # Trigger diagnostic publication
        node.publish_diagnostics()
        rclpy.spin_once(node, timeout_sec=0.5)

        assert len(received_diags) > 0
        diag_status = received_diags[0].status[0]
        assert diag_status.name == 'birobot: motion_planning_groups'
        level_val = ord(diag_status.level) if isinstance(diag_status.level, bytes) else int(diag_status.level)
        assert level_val == 0
        assert diag_status.message == 'All groups loaded successfully'

        # Trigger audit log event
        goals = {'arm_1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'arm_2': None}
        node.log_audit_event(
            planning_group='dual_arms',
            goals=goals,
            timeout_sec=30.0,
            status_code='SUCCESS',
            trajectory_states=100,
            message='Partial goal test'
        )
        rclpy.spin_once(node, timeout_sec=0.5)

        assert len(received_audits) > 0
        audit_payload = json.loads(received_audits[0].data)
        assert audit_payload['event_type'] == 'PLANNING_RESULT'
        assert audit_payload['planning_group'] == 'dual_arms'
        assert audit_payload['status_code'] == 'SUCCESS'
        assert audit_payload['trajectory_states'] == 100

        # Trigger TIMEOUT audit log event
        node.log_audit_event(
            planning_group='dual_arms',
            goals={'arm_1': 'crossed_goal', 'arm_2': 'crossed_goal'},
            timeout_sec=30.0,
            status_code='TIMEOUT',
            trajectory_states=0,
            message='Planner exhausted timeout'
        )
        rclpy.spin_once(node, timeout_sec=0.5)

        assert len(received_audits) == 2
        timeout_payload = json.loads(received_audits[1].data)
        assert timeout_payload['status_code'] == 'TIMEOUT'
        assert timeout_payload['trajectory_states'] == 0

        node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    test_motion_planning_monitor_diagnostics()
    print("ALL TESTS PASSED")
