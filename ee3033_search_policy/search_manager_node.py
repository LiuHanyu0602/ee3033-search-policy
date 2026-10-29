"""Closed-loop runtime search manager.

Navigation is allowed only when navigation_enabled is true and the mission
state is SEARCHING. The node owns its Nav2 goal, enabling safe cancellation.
"""
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .coverage_memory import CoverageMemory
from .feature_builder import build_candidate_features, make_shortlist, turn_angles
from .map_utils import GridGeometry, occupancy_array, world_to_grid
from .model import TinySearchPolicy
from .nav2_client import Nav2Client
from .policy_decision import select_with_fallback
from .runtime_viewpoints import generate_viewpoints_from_grid
from .target_stop_guard import TargetStopGuard


def make_pose(frame, stamp, x, y, yaw):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
    msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return msg


class SearchManager(Node):
    def __init__(self):
        super().__init__('search_manager')
        defaults = {
            'map_topic': '/map', 'mission_state_topic': '/mission/state',
            'target_found_topic': '/target_found', 'map_frame': 'map',
            'robot_frame': 'base_footprint',
            'planner_action': '/compute_path_to_pose',
            'navigate_action': '/navigate_to_pose', 'planner_id': 'GridBased',
            'navigation_enabled': False, 'return_home_enabled': True, 'weights_path': '',
            'single_goal_mode': True, 'max_navigation_goals': 1,
            'candidate_spacing_m': 0.75, 'candidate_clearance_m': 0.20,
            'camera_fov_deg': 70.0, 'camera_range_m': 3.0,
            'headings_per_position': 4, 'free_threshold': 20,
            'occupied_threshold': 65, 'shortlist_size': 32,
            'planning_candidates': 16, 'linear_speed_mps': 0.25,
            'angular_speed_rps': 0.8, 'observation_time_s': 0.25,
            'time_discount_s': 20.0, 'planning_timeout_s': 8.0,
            'navigation_timeout_s': 120.0, 'decision_period_s': 0.5,
            'coverage_target': 0.995, 'minimum_gain_m2': 0.01,
            'max_candidate_failures': 2, 'failure_cooldown_s': 60.0,
            'memory_path': '/tmp/ee3033_search_memory.npz',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.map_topic, self.mission_topic = str(p('map_topic')), str(p('mission_state_topic'))
        self.map_frame, self.robot_frame = str(p('map_frame')), str(p('robot_frame'))
        self.planner_id = str(p('planner_id'))
        self.navigation_enabled = bool(p('navigation_enabled'))
        self.return_home_enabled = bool(p('return_home_enabled'))
        self.single_goal_mode = bool(p('single_goal_mode'))
        self.max_navigation_goals = int(p('max_navigation_goals'))
        if self.max_navigation_goals < 1:
            raise ValueError('max_navigation_goals must be at least 1')
        self.spacing, self.clearance = float(p('candidate_spacing_m')), float(p('candidate_clearance_m'))
        self.camera_fov, self.camera_range = float(p('camera_fov_deg')), float(p('camera_range_m'))
        self.heading_count = int(p('headings_per_position'))
        self.free_threshold, self.occupied_threshold = int(p('free_threshold')), int(p('occupied_threshold'))
        self.shortlist_size, self.planning_candidates = int(p('shortlist_size')), int(p('planning_candidates'))
        self.linear_speed, self.angular_speed = float(p('linear_speed_mps')), float(p('angular_speed_rps'))
        self.observation_time, self.time_discount = float(p('observation_time_s')), float(p('time_discount_s'))
        self.planning_timeout, self.navigation_timeout = float(p('planning_timeout_s')), float(p('navigation_timeout_s'))
        self.coverage_target, self.minimum_gain_m2 = float(p('coverage_target')), float(p('minimum_gain_m2'))
        self.max_failures, self.failure_cooldown = int(p('max_candidate_failures')), float(p('failure_cooldown_s'))
        self.memory_path = str(p('memory_path'))

        weights = str(p('weights_path')).strip()
        if not weights:
            share = Path(get_package_share_directory('ee3033_search_policy'))
            weights = str(share / 'models' / 'tiny_search_policy_weights.npz')
        self.policy = TinySearchPolicy(weights)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav2 = Nav2Client(
            self, str(p('planner_action')), str(p('navigate_action')),
            self.navigation_enabled,
        )
        self.stop_guard = TargetStopGuard(
            self, topic=str(p('target_found_topic')), on_stop=self._target_stop,
        )
        self.mission_state, self.map_signature = 'IDLE', None
        self.geometry = self.candidates = self.memory = None
        self.failure_counts = self.failure_times = None
        self.plan_indices = np.empty(0, dtype=np.int32)
        self.path_distances = None
        self.plan_cursor = 0
        self.current_pose = self.active_plan = self.active_navigation = None
        self.selected_index = self.observation_deadline = None
        self.home_pose = None
        self.return_request = None
        self.return_pending = False
        self.return_terminal_verified = True
        self.return_started = False
        self.return_failed = False
        self.busy = self.finished = False
        self.retry_after = 0.0
        self.navigation_goal_count = 0
        self.operator_cancel_requested = False

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, '/search/status', latched)
        self.coverage_pub = self.create_publisher(Float32, '/search/coverage_fraction', latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/search/selected_goal', latched)
        self.complete_pub = self.create_publisher(Bool, '/search/finished', latched)
        self.return_pub = self.create_publisher(String, '/search/return_status', latched)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, latched)
        self.create_subscription(String, self.mission_topic, self._mission_cb, latched)
        self.create_service(Trigger, '/search/reset_memory', self._reset)
        self.create_service(Trigger, '/search/save_memory', self._save)
        self.create_service(Trigger, '/search/load_memory', self._load)
        self.create_service(Trigger, '/search/cancel_navigation', self._cancel_navigation)
        self.timer = self.create_timer(float(p('decision_period_s')), self._tick)
        self._status('WAITING_FOR_MAP')
        self.get_logger().warning(
            'Navigation is ' + ('ENABLED' if self.navigation_enabled else 'DISABLED')
        )
        if self.single_goal_mode:
            self.get_logger().warning(
                f'SINGLE-GOAL MODE: at most {self.max_navigation_goals} '
                'navigation goal(s) may be dispatched'
            )

    def _status(self, value):
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def _return_status(self, status):
        msg = String()
        msg.data = status
        self.return_pub.publish(msg)

    def _progress(self):
        msg = Float32()
        msg.data = float(self.memory.coverage_fraction) if self.memory else 0.0
        self.coverage_pub.publish(msg)
        done = Bool()
        done.data = self.finished
        self.complete_pub.publish(done)

    def _map_cb(self, msg):
        try:
            geometry = GridGeometry.from_message(msg)
            values = occupancy_array(msg)
            signature = (geometry, hash(values.tobytes()))
            if signature == self.map_signature:
                return
            if self.busy or self.active_navigation is not None:
                self.get_logger().warning('Map update deferred during an active decision')
                return
            candidates = generate_viewpoints_from_grid(
                values, geometry, self.spacing, self.clearance,
                self.camera_fov, self.camera_range, self.heading_count,
                self.free_threshold, self.occupied_threshold,
            )
            self.geometry, self.candidates = geometry, candidates
            self.memory = CoverageMemory(candidates.observable_mask, len(candidates))
            self.failure_counts = np.zeros(len(candidates), dtype=np.int32)
            self.failure_times = np.full(len(candidates), -np.inf)
            self.map_signature, self.finished, self.busy = signature, False, False
            self._progress()
            self._status('READY')
            self.get_logger().info(f'Generated {len(candidates)} candidate poses')
        except (TypeError, ValueError) as exc:
            self._status('MAP_ERROR')
            self.get_logger().error(f'Runtime map error: {exc}')

    def _mission_cb(self, msg):
        previous = self.mission_state
        self.mission_state = msg.data.strip().upper()
        if self.memory:
            if self.mission_state == 'PAUSED':
                self.memory.pause()
            elif self.mission_state == 'SEARCHING':
                self.memory.resume()
        if previous != self.mission_state:
            self.get_logger().info(f'Mission: {previous} -> {self.mission_state}')
        if self.mission_state == 'IDLE':
            self._cancel('mission reset')
            if self.return_request is not None and not self.return_request.future.done():
                self.return_request.cancel()
            self.home_pose = None
            self.return_pending = self.return_started = self.return_failed = False
            self.return_terminal_verified = True
            self._return_status('IDLE')
            if self.memory:
                self.memory.reset()
            if self.stop_guard.active_request is None:
                self.stop_guard.reset()
            self.finished, self.busy = False, False
            self.retry_after = 0.0
            self.navigation_goal_count = 0
            self.operator_cancel_requested = False
            self._progress()
        elif self.mission_state != 'SEARCHING':
            self._cancel('mission not searching')
        elif previous != 'SEARCHING':
            self.navigation_goal_count = 0
            self.operator_cancel_requested = False
            if self.home_pose is not None or self.stop_guard.stopped:
                return
            origin = self._robot_pose()
            if origin is not None:
                self.home_pose = tuple(origin)
                self._return_status('HOME_RECORDED')
                self.get_logger().info(f'Search home recorded: {self.home_pose}')

    def _robot_pose(self):
        if self.geometry is None:
            self._status('WAITING_FOR_MAP')
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self._status('WAITING_FOR_TF')
            self.get_logger().warning(f'TF unavailable: {exc}', throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        t = tf.transform.translation
        try:
            world_to_grid(t.x, t.y, self.geometry)
        except ValueError as exc:
            self._status('ROBOT_POSE_OUTSIDE_MAP')
            self.get_logger().error(
                f'Robot TF position is invalid for the occupancy map: {exc}; '
                'set a valid localization pose before planning',
                throttle_duration_sec=2.0,
            )
            return None
        return float(t.x), float(t.y), float(yaw)

    def _blocked(self):
        return ((self.failure_counts >= self.max_failures) &
                ((time.monotonic() - self.failure_times) < self.failure_cooldown))

    def _tick(self):
        self._progress()
        if self.return_pending:
            self._try_return_home()
            return
        if self.observation_deadline is not None:
            if time.monotonic() >= self.observation_deadline:
                self._finish_observation()
            return
        if (self.mission_state != 'SEARCHING' or self.finished or self.busy or
                self.operator_cancel_requested or self.memory is None or not self.stop_guard.may_dispatch() or
                time.monotonic() < self.retry_after):
            return
        if self.memory.coverage_fraction >= self.coverage_target:
            self._finish_search('COVERAGE_TARGET_REACHED')
            return
        pose = self._robot_pose()
        if pose is None:
            return
        elif self.home_pose is None:
            self.home_pose = tuple(pose)
            self._return_status('HOME_RECORDED')
        elif not self.nav2.planner_is_ready():
            self._status('WAITING_FOR_PLANNER')
        else:
            self._begin_decision(pose)

    def _begin_decision(self, pose):
        self.busy, self.current_pose = True, pose
        gains = self.memory.gains(self.candidates.visibility_masks)
        minimum = max(1, int(math.ceil(self.minimum_gain_m2 / self.geometry.resolution**2)))
        gains[gains < minimum] = 0
        euclidean = np.hypot(self.candidates.world_x-pose[0], self.candidates.world_y-pose[1])
        turns = turn_angles(pose[2], self.candidates.headings)
        indices, _, _ = make_shortlist(
            gains, euclidean, turns, self.linear_speed, self.angular_speed,
            self.observation_time, self.time_discount,
            max(self.shortlist_size, self.planning_candidates), self._blocked(),
        )
        if indices.size == 0:
            self._finish_search('NO_REMAINING_GAIN')
            return
        self.plan_indices = indices[:self.planning_candidates]
        self.path_distances = np.full(len(self.candidates), np.inf)
        self.plan_cursor = 0
        self._status('PLANNING_CANDIDATES')
        self._plan_next()

    def _plan_next(self):
        if self.plan_cursor >= self.plan_indices.size:
            self._choose()
            return
        index = int(self.plan_indices[self.plan_cursor])
        stamp = self.get_clock().now().to_msg()
        start = make_pose(self.map_frame, stamp, *self.current_pose)
        goal = make_pose(
            self.map_frame, stamp, self.candidates.world_x[index],
            self.candidates.world_y[index], self.candidates.headings[index],
        )
        try:
            req = self.nav2.compute_path(start, goal, self.planner_id, self.planning_timeout)
            self.active_plan = req
            req.future.add_done_callback(lambda future, i=index: self._plan_done(i, future))
        except (RuntimeError, ValueError) as exc:
            self._failure(index, str(exc))
            self.plan_cursor += 1
            self._plan_next()

    def _plan_done(self, index, future):
        self.active_plan = None
        try:
            result = future.result()
            if result.success:
                self.path_distances[index] = result.path_length_m
            elif result.accepted is False:
                self.busy = False
                self.retry_after = time.monotonic() + 5.0
                self._status('WAITING_FOR_PLANNER_ACTIVE')
                self.get_logger().error(
                    'Nav2 planner rejected the request; verify planner_server '
                    'lifecycle readiness before search',
                    throttle_duration_sec=2.0,
                )
                return
            else:
                self._failure(index, result.error)
        except Exception as exc:
            self._failure(index, str(exc))
        self.plan_cursor += 1
        self._plan_next()

    def _choose(self):
        gains = self.memory.gains(self.candidates.visibility_masks)
        turns = turn_angles(self.current_pose[2], self.candidates.headings)
        shortlist, _, utility = make_shortlist(
            gains, self.path_distances, turns, self.linear_speed,
            self.angular_speed, self.observation_time, self.time_discount,
            self.shortlist_size, self._blocked(),
        )
        if shortlist.size == 0:
            self._finish_search('NO_REACHABLE_GAIN')
            return
        euclidean = np.hypot(
            self.candidates.world_x-self.current_pose[0],
            self.candidates.world_y-self.current_pose[1],
        )
        visible_counts = np.count_nonzero(self.candidates.visibility_masks, axis=(1, 2))
        unseen = int(np.count_nonzero(self.memory.unseen_mask))
        searched = int(np.count_nonzero(self.memory.searched_mask & self.memory.observable_mask))
        features = build_candidate_features(
            shortlist, gains, unseen, searched, self.path_distances, turns,
            self.candidates.clearance_m, self.memory.visit_counts,
            self.candidates.visible_areas_m2, visible_counts, euclidean,
            self.geometry.resolution, self.linear_speed, self.angular_speed,
            self.observation_time,
        )
        index, scores, method = select_with_fallback(
            self.policy, features, shortlist, utility,
        )
        self.selected_index = int(index)
        goal = make_pose(
            self.map_frame, self.get_clock().now().to_msg(),
            self.candidates.world_x[index], self.candidates.world_y[index],
            self.candidates.headings[index],
        )
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'Selected {index} by {method}; path={self.path_distances[index]:.2f}m'
        )
        if not self.navigation_enabled:
            # Hold after one complete decision so dry-run does not repeatedly
            # query Nav2 against an unchanged state.
            self.busy = True
            self._status('DRY_RUN_SELECTION_READY')
            return
        if not self.stop_guard.may_dispatch():
            self.busy = False
            return
        if (self.single_goal_mode and
                self.navigation_goal_count >= self.max_navigation_goals):
            self._finish_search('SINGLE_GOAL_LIMIT_REACHED')
            return
        if not self.nav2.navigator_is_ready():
            self.busy = False
            self._status('WAITING_FOR_NAVIGATOR')
            return
        try:
            req = self.nav2.navigate_to_pose(goal, self.navigation_timeout)
            self.active_navigation = req
            self.navigation_goal_count += 1
            self.stop_guard.track_navigation(req)
            req.future.add_done_callback(lambda future, i=index: self._nav_done(i, future))
            self._status('NAVIGATING')
        except (RuntimeError, ValueError) as exc:
            self.active_navigation, self.busy = None, False
            self._failure(index, str(exc))

    def _nav_done(self, index, future):
        self.active_navigation = None
        if self.operator_cancel_requested:
            try:
                result = future.result()
                verified = (not result.timed_out and result.accepted is True and
                            result.status in (4, 5))
            except Exception:
                verified = False
            self.busy = False
            self.finished = True
            self._status('OPERATOR_STOPPED' if verified else 'OPERATOR_STOP_UNVERIFIED')
            self._progress()
            return
        if self.return_pending:
            try:
                result = future.result()
                self.return_terminal_verified = (not result.timed_out and result.accepted is True and
                    result.status in (4, 5))  # SUCCEEDED or CANCELED
            except Exception:
                self.return_terminal_verified = False
            self.busy = False
            self._try_return_home()
            return
        try:
            result = future.result()
            if result.success and not self.stop_guard.stopped:
                self.selected_index = index
                self.observation_deadline = time.monotonic() + self.observation_time
                self._status('OBSERVING')
                return
            self._failure(index, result.error)
        except Exception as exc:
            self._failure(index, str(exc))
        self.busy, self.selected_index = False, None
        if (self.single_goal_mode and
                self.navigation_goal_count >= self.max_navigation_goals):
            self._finish_search('SINGLE_GOAL_TEST_FAILED')

    def _finish_observation(self):
        index = self.selected_index
        self.observation_deadline = None
        if (index is not None and not self.stop_guard.stopped and
                self.mission_state == 'SEARCHING'):
            self.memory.mark_observation_complete(index, self.candidates.visibility_masks[index])
        self.selected_index, self.busy = None, False
        if (self.single_goal_mode and
                self.navigation_goal_count >= self.max_navigation_goals):
            self._finish_search('SINGLE_GOAL_TEST_COMPLETE')
            return
        self._status('READY')
        self._progress()

    def _failure(self, index, reason):
        self.failure_counts[index] += 1
        self.failure_times[index] = time.monotonic()
        self.get_logger().warning(f'Candidate {index} failed: {reason}')

    def _cancel(self, reason):
        req = self.active_navigation
        if req is not None and not req.future.done():
            self.get_logger().warning(f'Cancel navigation: {reason}')
            req.cancel()

    def _target_stop(self):
        if self.memory:
            self.memory.pause()
        self.observation_deadline, self.busy = None, False
        self._status('TARGET_FOUND')
        if self.return_home_enabled and self.navigation_enabled:
            self.return_pending = True
            self.return_terminal_verified = self.active_navigation is None
            self._return_status('WAITING_FOR_SEARCH_STOP')

    def _try_return_home(self):
        if self.return_started or self.return_failed or not self.return_pending:
            return
        if self.active_navigation is not None:
            return  # Nav2 cancellation must finish first
        if not self.return_terminal_verified:
            self.return_failed = True
            self._return_status('FAILED_UNVERIFIED_SEARCH_STOP')
            return
        if self.home_pose is None:
            self.return_failed = True
            self._return_status('FAILED_NO_HOME_POSE')
            return
        if not self.nav2.navigator_is_ready():
            self._return_status('WAITING_FOR_NAVIGATOR')
            return
        home = make_pose(self.map_frame, self.get_clock().now().to_msg(), *self.home_pose)
        try:
            self.return_request = self.nav2.navigate_to_pose(home, self.navigation_timeout)
            self.return_started = True
            self._return_status('RETURNING')
            self.return_request.future.add_done_callback(self._return_done)
        except (RuntimeError, ValueError) as exc:
            self.return_failed = True
            self._return_status('FAILED_SUBMISSION')
            self.get_logger().error(f'Return-home submission failed: {exc}')

    def _return_done(self, future):
        if self.operator_cancel_requested:
            try:
                result = future.result()
                verified = (not result.timed_out and result.accepted is True and
                            result.status in (4, 5))
            except Exception:
                verified = False
            self.return_pending = False
            self._return_status(
                'OPERATOR_STOPPED' if verified else 'OPERATOR_STOP_UNVERIFIED'
            )
            self._status(
                'OPERATOR_STOPPED' if verified else 'OPERATOR_STOP_UNVERIFIED'
            )
            return
        try:
            result = future.result()
            if result.success:
                self._return_status('SUCCEEDED')
                self._status('RETURN_HOME_SUCCEEDED')
            else:
                self._return_status('FAILED_NAVIGATION')
                self._status('RETURN_HOME_FAILED')
        except Exception as exc:
            self._return_status('FAILED_NAVIGATION')
            self.get_logger().error(f'Return-home result failed: {exc}')
        self.return_pending = False

    def _finish_search(self, reason):
        self.finished, self.busy = True, False
        self._status(reason)
        self._progress()

    def _cancel_navigation(self, request, response):
        del request
        self.operator_cancel_requested = True
        self.observation_deadline = None
        if self.memory:
            self.memory.pause()

        requests = []
        if self.active_navigation is not None and not self.active_navigation.future.done():
            requests.append(self.active_navigation)
        if self.return_request is not None and not self.return_request.future.done():
            requests.append(self.return_request)

        for active_request in requests:
            active_request.cancel()

        if requests:
            self._status('OPERATOR_CANCEL_REQUESTED')
            response.success = True
            response.message = (
                f'Cancellation requested for {len(requests)} active navigation goal(s); '
                'wait for /search/status terminal confirmation'
            )
        else:
            self.busy = False
            self.finished = True
            self._status('OPERATOR_STOPPED')
            self._progress()
            response.success = True
            response.message = 'No active navigation goal; future dispatch is inhibited'
        return response

    def _reset(self, request, response):
        del request
        if self.active_navigation is not None or self.return_pending or self.memory is None:
            response.success = False
            response.message = 'Navigation active or map unavailable'
            return response
        self.memory.reset()
        self.failure_counts.fill(0)
        self.failure_times.fill(-np.inf)
        self.finished, self.busy = False, False
        self.retry_after = 0.0
        self.navigation_goal_count = 0
        self.operator_cancel_requested = False
        if self.stop_guard.active_request is None:
            self.stop_guard.reset()
        self._progress()
        response.success, response.message = True, 'Search memory reset'
        return response

    def _save(self, request, response):
        del request
        try:
            if self.memory is None:
                raise ValueError('Map unavailable')
            self.memory.save(self.memory_path)
            response.success, response.message = True, f'Saved {self.memory_path}'
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _load(self, request, response):
        del request
        try:
            if self.active_navigation is not None or self.memory is None:
                raise ValueError('Navigation active or map unavailable')
            loaded = CoverageMemory.load(
                self.memory_path, expected_observable_mask=self.memory.observable_mask,
            )
            if loaded.visit_counts.size != len(self.candidates):
                raise ValueError('Candidate count mismatch')
            self.memory, self.finished = loaded, False
            self._progress()
            response.success, response.message = True, f'Loaded {self.memory_path}'
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def destroy_node(self):
        self.nav2.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SearchManager()
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
