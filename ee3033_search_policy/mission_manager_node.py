"""Mission-stage state machine for the two-stage EE3033 assessment."""

from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MissionState(str, Enum):
    IDLE = 'IDLE'
    MAPPING = 'MAPPING'
    WAIT_FOR_TARGET = 'WAIT_FOR_TARGET'
    SEARCHING = 'SEARCHING'
    PAUSED = 'PAUSED'
    TARGET_FOUND = 'TARGET_FOUND'
    FINISHED = 'FINISHED'


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state = MissionState.IDLE
        self.resume_state = MissionState.IDLE
        self.last_target_position = None

        self.state_pub = self.create_publisher(
            String, '/mission/state', latched
        )
        self.mapping_pub = self.create_publisher(
            Bool, '/mapping_complete', latched
        )
        self.search_pub = self.create_publisher(
            Bool, '/search_complete', latched
        )
        self.notification_pub = self.create_publisher(
            String, '/mission/notification', latched
        )

        self.create_subscription(
            Bool, '/target_found', self._target_found_callback, latched
        )
        self.create_subscription(
            PointStamped,
            '/target_position',
            self._target_position_callback,
            10,
        )
        self.create_subscription(
            Bool, '/search/finished', self._search_finished_callback, latched
        )

        services = {
            '/mission/start_mapping': self._start_mapping,
            '/mission/finish_mapping': self._finish_mapping,
            '/mission/start_search': self._start_search,
            '/mission/pause': self._pause,
            '/mission/resume': self._resume,
            '/mission/finish': self._finish,
            '/mission/reset': self._reset,
        }
        self.services = [
            self.create_service(Trigger, name, callback)
            for name, callback in services.items()
        ]
        self._publish_state('Mission manager ready')

    def _reply(self, response, success, message):
        response.success = success
        response.message = message
        return response

    def _set_state(self, state, notification):
        self.state = MissionState(state)
        self._publish_state(notification)

    def _publish_state(self, notification=''):
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        mapping = Bool()
        mapping.data = self.state in (
            MissionState.WAIT_FOR_TARGET,
            MissionState.SEARCHING,
            MissionState.PAUSED,
            MissionState.TARGET_FOUND,
            MissionState.FINISHED,
        )
        self.mapping_pub.publish(mapping)

        search = Bool()
        search.data = self.state in (
            MissionState.TARGET_FOUND,
            MissionState.FINISHED,
        )
        self.search_pub.publish(search)

        if notification:
            message = String()
            message.data = notification
            self.notification_pub.publish(message)
            self.get_logger().info(notification)

    def _start_mapping(self, request, response):
        del request
        if self.state not in (MissionState.IDLE, MissionState.FINISHED):
            return self._reply(
                response, False, f'Cannot start mapping from {self.state.value}'
            )
        self._set_state(
            MissionState.MAPPING,
            'MAPPING_STARTED',
        )
        return self._reply(response, True, 'Mapping stage started')

    def _finish_mapping(self, request, response):
        del request
        if self.state != MissionState.MAPPING:
            return self._reply(
                response, False, 'Mapping is not currently active'
            )
        self._set_state(
            MissionState.WAIT_FOR_TARGET,
            'MAPPING_COMPLETE: place target, then call /mission/start_search',
        )
        return self._reply(response, True, 'Mapping marked complete')

    def _start_search(self, request, response):
        del request
        if self.state != MissionState.WAIT_FOR_TARGET:
            return self._reply(
                response,
                False,
                f'Cannot start search from {self.state.value}',
            )
        self._set_state(MissionState.SEARCHING, 'SEARCH_STARTED')
        return self._reply(response, True, 'Target search started')

    def _pause(self, request, response):
        del request
        if self.state not in (
            MissionState.MAPPING,
            MissionState.SEARCHING,
        ):
            return self._reply(
                response, False, f'Cannot pause from {self.state.value}'
            )
        self.resume_state = self.state
        self._set_state(MissionState.PAUSED, 'MISSION_PAUSED')
        return self._reply(response, True, 'Mission paused')

    def _resume(self, request, response):
        del request
        if self.state != MissionState.PAUSED:
            return self._reply(response, False, 'Mission is not paused')
        destination = self.resume_state
        if destination not in (
            MissionState.MAPPING,
            MissionState.SEARCHING,
        ):
            destination = MissionState.IDLE
        self._set_state(destination, f'MISSION_RESUMED:{destination.value}')
        return self._reply(response, True, 'Mission resumed')

    def _finish(self, request, response):
        del request
        self._set_state(MissionState.FINISHED, 'MISSION_FINISHED')
        return self._reply(response, True, 'Mission finished')

    def _reset(self, request, response):
        del request
        self.last_target_position = None
        self.resume_state = MissionState.IDLE
        self._set_state(MissionState.IDLE, 'MISSION_RESET')
        return self._reply(response, True, 'Mission reset')

    def _target_position_callback(self, msg):
        self.last_target_position = msg

    def _search_finished_callback(self, msg):
        if msg.data and self.state == MissionState.SEARCHING:
            self._set_state(
                MissionState.FINISHED,
                'SEARCH_COMPLETE: target was not detected in observable space',
            )

    def _target_found_callback(self, msg):
        if not msg.data or self.state != MissionState.SEARCHING:
            return
        self._set_state(
            MissionState.TARGET_FOUND,
            'TARGET_FOUND: stop navigation and report target position',
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
