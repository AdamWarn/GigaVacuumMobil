"""Systematic cleaning pattern that triggers the coverage executor."""

from __future__ import annotations

from typing import Callable

from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class SystematicCleaningPattern:
    """Lightweight pattern that tells the coverage executor to begin."""

    def __init__(
        self,
        node: Node,
        status_cb: Callable[[str], None],
        complete_cb: Callable[[str, bool, str], None],
    ) -> None:
        self._node = node
        self._status_cb = status_cb
        self._complete_cb = complete_cb
        self.name = 'systematic_cleaning'

        self._service_name = self._param('cleaning.start_service', '/coverage_executor/start_coverage')
        self._enable_topic = self._param('cleaning.enable_topic', '/coverage_enable')
        self._auto_finish = self._param_bool('cleaning.auto_finish', True)

        self._client = node.create_client(Trigger, self._service_name)
        self._enable_pub = node.create_publisher(Bool, self._enable_topic, 10) if self._enable_topic else None

        self._active = False
        self._pending_future = None

    def start(self) -> bool:
        if self._active:
            self._status('Systematic cleaning already running.')
            return False
        self._active = True
        self._status('Starting systematic cleaning pattern.')
        if not self._client.service_is_ready():
            self._status('Waiting for coverage executor service...')
            self._client.wait_for_service()
        request = Trigger.Request()
        self._pending_future = self._client.call_async(request)
        self._pending_future.add_done_callback(self._handle_response)
        if self._enable_pub is not None:
            msg = Bool()
            msg.data = True
            self._enable_pub.publish(msg)
        return True

    def stop(self, reason: str = 'stopped') -> None:
        if not self._active:
            return
        self._active = False
        self._pending_future = None
        self._status(f'Systematic cleaning pattern stopped ({reason}).')
        self._complete(False, reason)

    def _handle_response(self, future) -> None:
        if not self._active:
            return
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime diagnostics
            self._status(f'Coverage executor service failed: {exc}')
            self._finish(False, 'service_error')
            return
        if response.success:
            self._status('Coverage executor acknowledged start request.')
            self._finish(True, 'coverage_started' if self._auto_finish else 'running')
        else:
            self._status(f'Coverage executor rejected start request: {response.message}')
            self._finish(False, 'coverage_rejected')

    def _finish(self, success: bool, reason: str) -> None:
        if not self._auto_finish and success:
            # Keep pattern marked active so manager waits for manual stop.
            return
        self._active = False
        self._complete(success, reason)

    def _complete(self, success: bool, reason: str) -> None:
        if self._complete_cb:
            self._complete_cb(self.name, success, reason)

    def _status(self, text: str) -> None:
        if self._status_cb:
            self._status_cb(f'[{self.name}] {text}')

    def _param(self, name: str, default):
        if self._node.has_parameter(name):
            return self._node.get_parameter(name).value
        self._node.declare_parameter(name, default)
        return default

    def _param_bool(self, name: str, default: bool) -> bool:
        value = self._param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'on', 'yes')
        return bool(value)
