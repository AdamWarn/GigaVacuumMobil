"""Executor node that feeds coverage plan batches into Nav2's NavigateThroughPoses."""

from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class CoverageExecutorNode(Node):
    def __init__(self):
        super().__init__('coverage_executor', automatically_declare_parameters_from_overrides=True)

        self._declare_if_needed('plan_topic', 'coverage_plan')
        self._declare_if_needed('action_name', 'navigate_through_poses')
        self._declare_if_needed('poses_per_goal', 1)
        self._declare_if_needed('loop_plan', False)
        self._declare_if_needed('auto_start', True)
        self._declare_if_needed('enable_topic', '')

        plan_topic = self.get_parameter('plan_topic').value
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._plan_sub = self.create_subscription(Path, plan_topic, self._plan_callback, qos)
        self._nav_client = ActionClient(self, NavigateThroughPoses, self.get_parameter('action_name').value)

        self._pending_chunks: List[List[PoseStamped]] = []
        self._current_goal_handle = None
        self._loop_plan = bool(self.get_parameter('loop_plan').value)
        self._stashed_plan = []
        self._wait_timer = None
        self._enabled = bool(self.get_parameter('auto_start').value)

        enable_topic = self.get_parameter('enable_topic').value
        if enable_topic:
            self._enable_sub = self.create_subscription(Bool, enable_topic, self._enable_callback, 10)
        else:
            self._enable_sub = None

        self._start_srv = self.create_service(Trigger, 'start_coverage', self._handle_start_request)

    def _declare_if_needed(self, name, value):
        if not self.has_parameter(name):
            self.declare_parameter(name, value)

    # ------------------------------------------------------------------
    def _plan_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn('Received empty coverage plan, ignoring.')
            return

        self.get_logger().info(f'Received coverage plan with {len(msg.poses)} poses.')
        self._stashed_plan = list(msg.poses)
        self._pending_chunks = self._chunk_plan(self._stashed_plan)

        if not self._enabled:
            self.get_logger().info('Coverage executor is disabled; waiting for enable signal before starting plan.')
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 action server...')
            if self._wait_timer is None:
                self._wait_timer = self.create_timer(0.5, self._check_server_ready)
            return

        self._send_next_chunk()

    def _check_server_ready(self):
        if not self._nav_client.server_is_ready():
            return
        if not self._enabled:
            return

        self.get_logger().info('Nav2 action server is ready; starting coverage execution.')
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None
        self._send_next_chunk()

    def _chunk_plan(self, poses):
        chunk_size = max(1, int(self.get_parameter('poses_per_goal').value))
        return [poses[i:i + chunk_size] for i in range(0, len(poses), chunk_size)]

    def _send_next_chunk(self):
        if not self._enabled:
            self.get_logger().debug('Coverage executor paused; skipping goal dispatch.')
            return

        if not self._nav_client.server_is_ready():
            if self._wait_timer is None:
                self._wait_timer = self.create_timer(0.5, self._check_server_ready)
            return

        if not self._pending_chunks:
            if self._loop_plan and self._stashed_plan:
                self.get_logger().info('Looping plan from start.')
                self._pending_chunks = self._chunk_plan(self._stashed_plan)
            else:
                self.get_logger().info('Coverage plan exhausted.')
                return

        if self._current_goal_handle is not None:
            self.get_logger().debug('Waiting for current goal to finish before sending next chunk.')
            return

        chunk = self._pending_chunks.pop(0)
        goal = NavigateThroughPoses.Goal()
        goal.poses = chunk

        self.get_logger().info(f'Sending NavigateThroughPoses goal with {len(chunk)} poses.')
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected coverage goal.')
            self._current_goal_handle = None
            return

        self.get_logger().info('Coverage segment accepted by Nav2.')
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Coverage segment finished successfully.')
            self._current_goal_handle = None
            self._send_next_chunk()
            return

        self.get_logger().error(f'Coverage segment failed with status={result.status}.')
        self._current_goal_handle = None

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Nav2 distance remaining {feedback.distance_remaining:.2f} m, "
            f"recoveries {feedback.number_of_recoveries}"
        )

    # ------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data:
            self._set_enabled(True, source='topic')

    def _handle_start_request(self, _request, response: Trigger.Response):
        self._set_enabled(True, source='service')
        response.success = True
        response.message = 'Coverage execution enabled.'
        return response

    def _set_enabled(self, value: bool, source: str = 'external') -> None:
        if value and not self._enabled:
            self._enabled = True
            self.get_logger().info(f'Coverage executor enabled via {source}.')
            if self._pending_chunks:
                self._send_next_chunk()
        elif not value and self._enabled:
            self._enabled = False
            self.get_logger().info(f'Coverage executor disabled via {source}.')


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
