"""Stop-on-target integration for ee3033-search-policy runtime-policy-core.

Attach to the *same ROS node that owns* the active Nav2 ActionRequest.
This module does not command motors and cannot cancel goals owned by another node.
"""
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool


class TargetStopGuard:
    def __init__(self, node, *, topic='/target_found', on_stop=None):
        self.node = node
        self.stopped = False
        self.active_request = None
        self._cancel_requested_for = None
        self._on_stop = on_stop
        self.subscription = node.create_subscription(Bool, topic, self._on_target_found, 10)

    def may_dispatch(self):
        """Mandatory gate before every candidate-selection/goal-submission step."""
        return not self.stopped

    def track_navigation(self, request):
        """Call immediately after nav2.navigate_to_pose(...), before any awaits."""
        self.active_request = request
        request.future.add_done_callback(lambda future, req=request: self._on_terminal(req, future))
        # Handles the target-found callback racing with goal submission.
        if self.stopped:
            self._cancel_active()
        return request

    def _on_target_found(self, msg):
        if not msg.data or self.stopped:
            return
        self.stopped = True  # latch; cannot send further waypoints
        self.node.get_logger().warn('TARGET FOUND: search locked; canceling navigation')
        if self._on_stop is not None:
            self._on_stop()  # set search-loop state / pause coverage updates
        self._cancel_active()

    def _cancel_active(self):
        request = self.active_request
        if request is None or request is self._cancel_requested_for:
            return
        if request.future.done():
            return
        self._cancel_requested_for = request
        try:
            cancellation = request.cancel()  # not request.future.cancel()
            cancellation.add_done_callback(self._on_cancel_ack)
        except Exception as exc:
            self.node.get_logger().error(f'Cancel submission failed: {exc}')

    def _on_cancel_ack(self, future):
        try:
            ack = bool(future.result())
        except Exception as exc:
            self.node.get_logger().error(f'Cancel acknowledgement error: {exc}')
            return
        if ack:
            self.node.get_logger().warn('Nav2 acknowledged cancellation; awaiting terminal result')
        else:
            self.node.get_logger().error('Nav2 did not acknowledge cancellation; verify robot state')

    def _on_terminal(self, request, future):
        if self.active_request is request:
            self.active_request = None
        try:
            result = future.result()
        except Exception as exc:
            self.node.get_logger().error(f'Navigation result unavailable: {exc}')
            return
        if self.stopped:
            if result.status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().warn('Nav2 reports navigation goal CANCELED')
            elif result.status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().warn('Nav2 goal finished before cancellation took effect')
            else:
                self.node.get_logger().error(
                    f'Nav2 terminal/local status={result.status}, error={result.error!r}; '
                    'robot stop not proven'
                )

    def reset(self):
        """Reset only as part of an explicit, stationary new mission."""
        if self.active_request is not None and not self.active_request.future.done():
            raise RuntimeError('Cannot reset while a navigation request is active')
        self.stopped = False
        self._cancel_requested_for = None
