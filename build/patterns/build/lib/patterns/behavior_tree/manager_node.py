"""High-level pattern manager with a simple behavior tree."""

from __future__ import annotations

import json
from typing import Callable, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from patterns.patterns.exploration.nodes.frontier_supervisor import ExplorationPattern
from patterns.patterns.systematic_cleaning.nodes.coverage_trigger import SystematicCleaningPattern


class PatternManagerNode(Node):
    """Coordinates exploration/cleaning patterns via a small BT-like flow."""

    def __init__(self) -> None:
        super().__init__('pattern_manager')

        # Declare shared parameters so users can override them from launch files.
        self._declare_pattern_parameters()

        self.declare_parameter('auto_mode', False)
        self.declare_parameter('auto_start_delay', 5.0)
        self.declare_parameter('pattern_sequence', ['exploration', 'systematic_cleaning'])
        self.declare_parameter('command_topic', '/pattern_manager/command')
        self.declare_parameter('status_topic', '/pattern_manager/status')

        self._auto_mode = self._get_bool_param('auto_mode', False)
        self._auto_delay = float(self.get_parameter('auto_start_delay').value)
        self._sequence = self._parse_sequence(self.get_parameter('pattern_sequence').value)
        self._sequence_index = 0

        self._status_topic = self.get_parameter('status_topic').value
        self._command_topic = self.get_parameter('command_topic').value

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._command_sub = self.create_subscription(String, self._command_topic, self._command_callback, 10)
        self._advance_srv = self.create_service(Trigger, 'advance_pattern', self._handle_advance_request)

        self._patterns = self._build_patterns()
        self._current_pattern: Optional[str] = None

        self._auto_timer = None
        self._status('Pattern manager ready (auto_mode=%s, command_topic=%s).' % (self._auto_mode, self._command_topic))

        if self._auto_mode and self._sequence:
            self._status('Auto mode enabled; starting sequence shortly.')
            self._auto_timer = self.create_timer(self._auto_delay, self._auto_begin)
        else:
            self._status('Manual mode: publish pattern names on %s to start them.' % self._command_topic)

    # ------------------------------------------------------------------
    def _declare_pattern_parameters(self) -> None:
        # Exploration namespace
        self.declare_parameter('exploration.map_topic', '/stable_map')
        self.declare_parameter('exploration.pose_topic', '/amcl_pose')
        self.declare_parameter('exploration.navigate_action', 'navigate_to_pose')
        self.declare_parameter('exploration.map_ready_topic', '/map_ready')
        self.declare_parameter('exploration.edge_band_width', 0.3)
        self.declare_parameter('exploration.min_edge_fraction', 0.7)
        self.declare_parameter('exploration.max_unknown_ratio', 0.18)
        self.declare_parameter('exploration.fallback_unknown_ratio', 0.3)
        self.declare_parameter('exploration.frontier_clearance', 0.25)
        self.declare_parameter('exploration.frontier_retry_limit', 5)
        self.declare_parameter('exploration.goal_timeout', 120.0)
        self.declare_parameter('exploration.analysis_period', 3.0)
        self.declare_parameter('exploration.occupied_threshold', 0.6)
        self.declare_parameter('exploration.free_threshold', 0.3)
        self.declare_parameter('exploration.max_goal_distance', 10.0)
        self.declare_parameter('exploration.behavior_tree', '')

        # Cleaning namespace
        self.declare_parameter('cleaning.start_service', '/coverage_executor/start_coverage')
        self.declare_parameter('cleaning.enable_topic', '/coverage_enable')
        self.declare_parameter('cleaning.auto_finish', True)

    def _build_patterns(self) -> Dict[str, object]:
        status_cb = self._status
        return {
            'exploration': ExplorationPattern(self, status_cb, self._on_pattern_complete),
            'systematic_cleaning': SystematicCleaningPattern(self, status_cb, self._on_pattern_complete),
        }

    # ------------------------------------------------------------------
    def _parse_sequence(self, raw_value) -> List[str]:
        if isinstance(raw_value, list):
            return [str(item) for item in raw_value]
        if isinstance(raw_value, str):
            try:
                parsed = json.loads(raw_value)
                if isinstance(parsed, list):
                    return [str(item) for item in parsed]
            except json.JSONDecodeError:
                self.get_logger().warn('Failed to parse pattern_sequence "%s"; falling back to defaults.', raw_value)
        return ['exploration', 'systematic_cleaning']

    def _auto_begin(self) -> None:
        if not self._auto_mode or not self._sequence:
            return
        if self._auto_timer is not None:
            self._auto_timer.cancel()
            self._auto_timer = None
        if self._current_pattern is None:
            self._start_pattern(self._sequence[self._sequence_index])

    # ------------------------------------------------------------------
    def _command_callback(self, msg: String) -> None:
        text = (msg.data or '').strip()
        if not text:
            return
        parts = text.split()
        cmd = parts[0].lower()
        args = parts[1:]

        if cmd in ('start', 'run') and args:
            self._start_pattern(args[0])
        elif cmd == 'stop':
            self._stop_current_pattern('manual_stop')
        elif cmd == 'next':
            self._advance_sequence(loop=True)
        elif cmd == 'auto':
            self._set_auto_mode(True)
        elif cmd in ('manual', 'man'):  # allow quick toggle
            self._set_auto_mode(False)
        elif cmd in self._patterns:
            self._start_pattern(cmd)
        else:
            self._status('Unknown command: %s' % text)

    def _handle_advance_request(self, _request, response: Trigger.Response) -> Trigger.Response:
        self._advance_sequence(loop=True)
        response.success = True
        response.message = 'Advanced pattern sequence.'
        return response

    # ------------------------------------------------------------------
    def _start_pattern(self, name: str) -> None:
        pattern = self._patterns.get(name)
        if pattern is None:
            self._status('Pattern "%s" not found.' % name)
            return
        if self._current_pattern and self._current_pattern != name:
            self._stop_current_pattern('preempted')
        started = pattern.start()
        if started:
            self._current_pattern = name
            self._status('Pattern %s started.' % name)
        else:
            self._status('Pattern %s already active.' % name)
            self._current_pattern = name

    def _stop_current_pattern(self, reason: str) -> None:
        if not self._current_pattern:
            return
        pattern = self._patterns.get(self._current_pattern)
        if pattern is None:
            self._current_pattern = None
            return
        if hasattr(pattern, 'stop'):
            pattern.stop(reason)
        self._current_pattern = None

    def _advance_sequence(self, *, loop: bool) -> None:
        if not self._sequence:
            self._status('No pattern sequence configured.')
            return
        next_index = self._sequence_index + 1
        if next_index >= len(self._sequence):
            if not loop:
                self._status('Pattern sequence finished; awaiting manual command.')
                return
            next_index = 0
        self._sequence_index = next_index
        self._start_pattern(self._sequence[self._sequence_index])

    def _on_pattern_complete(self, name: str, success: bool, reason: str) -> None:
        if self._current_pattern == name:
            self._current_pattern = None
        self._status('Pattern %s completed (%s, success=%s).' % (name, reason, success))
        if self._auto_mode and success and self._sequence:
            self._advance_sequence(loop=False)

    def _set_auto_mode(self, enabled: bool) -> None:
        self._auto_mode = enabled
        if enabled:
            self._status('Auto mode enabled.')
            self._sequence_index = 0
            if not self._current_pattern and self._sequence:
                self._start_pattern(self._sequence[0])
        else:
            self._status('Manual mode enabled.')

    # ------------------------------------------------------------------
    def _status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    # ------------------------------------------------------------------
    def _get_bool_param(self, name: str, default: bool) -> bool:
        """Retrieve a parameter that might arrive as bool, int, or string."""
        value = self.get_parameter(name).value if self.has_parameter(name) else None
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            return lowered in ('1', 'true', 'on', 'yes')
        return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatternManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()