"""Existing MLP search with a short-lived coordinator lease and stop handshake.

Run instead of search_manager, never alongside it. No direct velocity publisher.
"""
import json
import math
import time
import uuid
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from .search_manager_node import SearchManager, make_pose
from .balance_observation_gate import navigation_terminal_known


class GuardedSearchManager(SearchManager):
    def __init__(self):
        self.lease = None
        self.lease_receipt = 0.
        self.unknown_terminal = False
        self.foreign_active = False
        self.pending_action_status = None
        self.last_scan = None
        self.last_scan_stamp = None
        self.boot = str(uuid.uuid4())
        self.last_status = 'STARTING'
        self.last_terminal = 'NO_GOAL_DISPATCHED_BY_THIS_PROCESS'
        self.decision_serial = 0
        super().__init__()
        defaults = dict(lease_timeout_s=.6, tf_max_age_s=.3,
                        scan_max_age_s=.5, scan_topic='/scan',
                        exclusive_navigation_verified=False,
                        robot_geometry_verified=False)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.lease_timeout = float(p('lease_timeout_s'))
        self.tf_age = float(p('tf_max_age_s'))
        self.scan_age = float(p('scan_max_age_s'))
        self.exclusive = bool(p('exclusive_navigation_verified'))
        self.robot_verified = bool(p('robot_geometry_verified'))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.lease_timeout,self.tf_age,self.scan_age)):
            raise ValueError('Freshness limits must be positive')
        if self.return_home_enabled:
            raise ValueError('This validation workflow requires return_home_enabled=false')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.runtime_pub = self.create_publisher(String, '/search/runtime_state', latched)
        self.create_subscription(String, '/mission/lease', self._lease_cb, latched)
        self.create_subscription(LaserScan, str(p('scan_topic')), self._scan_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        action_name = str(self.get_parameter('navigate_action').value).rstrip('/')
        self.create_subscription(GoalStatusArray, action_name+'/_action/status',
                                 self._action_status, latched)
        self.create_timer(.1, self._watchdog)

    def _status(self, value):
        self.last_status = value
        super()._status(value)

    def _mission_cb(self, msg):
        # Legacy String is for displays and adapter; cannot authorize dispatch.
        pass

    def _lease_cb(self, msg):
        try:
            d = json.loads(msg.data)
            now = self.get_clock().now().nanoseconds*1e-9
            if not d['boot'] or not 0 <= now-float(d['stamp']) <= self.lease_timeout:
                return
            old = self.lease
            if old and d['boot'] != old['boot']:
                # Coordinator replacement is not a rearm instruction.
                self.unknown_terminal = True
            previous = self.mission_state
            self.lease, self.lease_receipt = d, time.monotonic()
            self.mission_state = str(d['state'])
            if previous != self.mission_state:
                self.decision_serial += 1
                if self.active_plan is not None and not self.active_plan.future.done():
                    self.active_plan.cancel()
                self.active_plan = None
            if self.memory:
                if self.mission_state == 'SEARCHING' and not self.operator_cancel_requested:
                    self.memory.resume()
                else:
                    self.memory.pause()
            if self.mission_state != 'SEARCHING':
                self.observation_deadline = None
                self._cancel('coordinator inhibits dispatch')
            elif previous != 'SEARCHING' and not self.unknown_terminal:
                # Resume preserves budget. A fresh configured episode resets it.
                if old and d.get('episode') != old.get('episode'):
                    self.navigation_goal_count = 0
                if previous == 'WAIT_FOR_TARGET':
                    self.finished = False
                self.busy = False
        except (ValueError, TypeError, KeyError):
            return

    def _scan_cb(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        if (stamp > 0 and msg.header.frame_id and msg.ranges and
                (self.last_scan_stamp is None or stamp > self.last_scan_stamp) and
                0 <= now-stamp <= self.scan_age):
            self.last_scan_stamp, self.last_scan = stamp, time.monotonic()

    def _action_status(self, msg):
        self.pending_action_status = msg
        own = self.active_navigation
        handle = getattr(own, '_goal_handle', None)
        if own is not None and handle is None:
            # Status can precede the goal-acceptance response. No new dispatch is
            # possible with an owned request outstanding; compare UUIDs as soon
            # as acceptance completes instead of misclassifying our own goal.
            return
        own_id = tuple(handle.goal_id.uuid) if handle is not None else None
        foreign = [s for s in msg.status_list if s.status in (1, 2, 3) and
                   tuple(s.goal_info.goal_id.uuid) != own_id]
        # During goal acceptance own UUID may not yet be populated. Inhibit now;
        # later matching feedback may clear foreign_active, never unknown latch.
        self.foreign_active = bool(foreign)

    def _healthy(self):
        now = self.get_clock().now().nanoseconds*1e-9
        steady = time.monotonic()
        if self.unknown_terminal:
            return False, 'REMOTE_TERMINAL_UNKNOWN'
        if self.foreign_active:
            return False, 'OTHER_NAVIGATION_GOAL_ACTIVE'
        if not self.exclusive or not self.robot_verified:
            return False, 'ROBOT_PREFLIGHT_NOT_VERIFIED'
        if (not self.lease or steady-self.lease_receipt > self.lease_timeout or
                not 0 <= now-float(self.lease['stamp']) <= self.lease_timeout):
            return False, 'COORDINATOR_LEASE_STALE'
        if self.memory is None or self.geometry is None:
            return False, 'WAITING_FOR_MAP'
        if (self.last_scan is None or steady-self.last_scan > self.scan_age or
                not 0 <= now-self.last_scan_stamp <= self.scan_age):
            return False, 'SCAN_STALE'
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            stamp = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            t, q = tf.transform.translation, tf.transform.rotation
            if (not 0 <= now-stamp <= self.tf_age or
                    not all(math.isfinite(v) for v in (t.x,t.y,t.z,q.x,q.y,q.z,q.w)) or
                    abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-3):
                return False, 'TF_STALE_OR_INVALID'
        except Exception:
            return False, 'TF_UNAVAILABLE'
        if not self.nav2.planner_is_ready():
            return False, 'PLANNER_UNAVAILABLE'
        if self.navigation_enabled and not self.nav2.navigator_is_ready():
            return False, 'NAVIGATOR_UNAVAILABLE'
        return True, 'READY'

    def _dispatch_allowed(self):
        return (self.mission_state == 'SEARCHING' and self.lease is not None and
                bool(self.lease.get('episode')) and not self.operator_cancel_requested and
                not self.finished and self._healthy()[0])

    def _tick(self):
        if not hasattr(self, 'lease_timeout'):
            return
        if not self._dispatch_allowed():
            return
        super()._tick()

    def _plan_next(self):
        if not self._dispatch_allowed():
            self.busy = False
            return
        # Capture generation on each async callback so a paused old decision
        # cannot mutate the state of a later resumed decision.
        if self.plan_cursor >= self.plan_indices.size:
            self._choose()
            return
        index = int(self.plan_indices[self.plan_cursor])
        stamp = self.get_clock().now().to_msg()
        start = make_pose(self.map_frame,stamp,*self.current_pose)
        goal = make_pose(self.map_frame,stamp,self.candidates.world_x[index],
                         self.candidates.world_y[index],self.candidates.headings[index])
        serial = self.decision_serial
        try:
            req = self.nav2.compute_path(start,goal,self.planner_id,self.planning_timeout)
            self.active_plan = req
            req.future.add_done_callback(lambda future,i=index,s=serial: self._generation_plan_done(i,s,future))
        except (RuntimeError,ValueError) as exc:
            self._failure(index,str(exc))
            self.plan_cursor += 1
            self._plan_next()

    def _generation_plan_done(self,index,serial,future):
        if serial != self.decision_serial:
            return
        super()._plan_done(index,future)

    def _choose(self):
        # Covers late planning callbacks after a target/cancel/lease expiration.
        if not self._dispatch_allowed():
            self.busy = False
            return
        super()._choose()

    def _nav_done(self, index, future):
        try:
            r = future.result()
            known = navigation_terminal_known(r)
            self.last_terminal = ('REJECTED' if r.accepted is False else str(r.status))
        except Exception:
            known = False
        if not known:
            self.unknown_terminal = True
            self.active_navigation = None
            self.busy, self.finished = False, True
            self._status('REMOTE_TERMINAL_UNKNOWN')
            return
        if self.mission_state != 'SEARCHING':
            self.active_navigation, self.busy = None, False
            self.observation_deadline = None
            return
        super()._nav_done(index, future)

    def _reset(self, request, response):
        if (self.unknown_terminal or self.foreign_active or
                self.mission_state != 'IDLE' or self.active_navigation is not None):
            response.success, response.message = False, 'Reset refused until idle and verified terminal'
            return response
        return super()._reset(request, response)

    def _load(self, request, response):
        if self.mission_state not in ('IDLE', 'WAIT_FOR_TARGET') or self.unknown_terminal:
            response.success, response.message = False, 'Load refused in active/unknown state'
            return response
        return super()._load(request, response)

    def _finish_observation(self):
        # Arrival success plus a timer alone does not prove a camera observation.
        pose = self._robot_pose()
        index = self.selected_index
        source_ready = bool(self.lease and self.lease.get('camera_ready'))
        valid = pose is not None and index is not None and source_ready and self._healthy()[0]
        if valid:
            distance = math.hypot(pose[0]-self.candidates.world_x[index],
                                  pose[1]-self.candidates.world_y[index])
            angle = pose[2]-self.candidates.headings[index]
            angle = abs(math.atan2(math.sin(angle), math.cos(angle)))
            valid = distance <= .15 and angle <= .20
        if not valid:
            self.observation_deadline = None
            self.selected_index, self.busy = None, False
            if index is not None:
                self._failure(index, 'Observation pose/camera readiness not verified')
            if self.single_goal_mode and self.navigation_goal_count >= self.max_navigation_goals:
                self._finish_search('SINGLE_GOAL_OBSERVATION_UNVERIFIED')
            return
        super()._finish_observation()

    def _watchdog(self):
        if self.pending_action_status is not None:
            self._action_status(self.pending_action_status)
        healthy, reason = self._healthy()
        if not healthy:
            self.observation_deadline = None
            self._cancel(reason)
            if self.mission_state == 'SEARCHING':
                # Do not resume automatically after a sensor/network outage.
                self.operator_cancel_requested = True
                self._status(reason)
        active = self.active_navigation
        quiescent = active is None and not self.unknown_terminal and not self.foreign_active
        self.runtime_pub.publish(String(data=json.dumps(dict(
            boot=self.boot, mission_boot=self.lease.get('boot','') if self.lease else '',
            episode=self.lease.get('episode','') if self.lease else '',
            stop_token=self.lease.get('stop_token','') if self.lease else '',
            stamp=self.get_clock().now().nanoseconds*1e-9,
            ready=healthy, reason=reason, quiescent=quiescent,
            terminal=self.last_terminal, unknown_terminal=self.unknown_terminal,
            navigation_enabled=self.navigation_enabled,
            operator_stop=self.operator_cancel_requested, finished=self.finished,
            status=self.last_status, goal_count=self.navigation_goal_count))))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GuardedSearchManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
