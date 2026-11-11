"""Episode-aware target mission. Candidate coordinates are never final reports."""
import json
import math
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from ee3033_search_policy_interfaces.msg import TargetObservation, TargetEstimate
from .balance_observation_gate import (
    MotionInterruptGate, MotionInterruptLimits, estimate_is_reportable,
)


class TargetMissionCoordinator(Node):
    def __init__(self):
        super().__init__('target_mission_coordinator')
        defaults = dict(odom_topic='/odom', odom_frame='odom_combined',
                        motion_interrupt_verified=False,
                        dynamic_camera_tf_verified=False,
                        stop_timeout_s=12., localization_timeout_s=15.,
                        input_max_age_s=.5, maximum_variance_m2=.25,
                        minimum_report_frames=3, report_consistency_m=.10,
                        mission_timeout_s=300., detector_fault_grace_s=0.)
        for k, v in vars(MotionInterruptLimits()).items():
            defaults['motion_interrupt_'+k] = v
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.gate = MotionInterruptGate(MotionInterruptLimits(**{
            k: p('motion_interrupt_'+k) for k in vars(MotionInterruptLimits())}))
        self.verified = bool(p('motion_interrupt_verified'))
        self.camera_motion_verified = bool(p('dynamic_camera_tf_verified'))
        self.odom_frame = str(p('odom_frame'))
        self.age = float(p('input_max_age_s'))
        self.stop_timeout = float(p('stop_timeout_s'))
        self.loc_timeout = float(p('localization_timeout_s'))
        self.variance = float(p('maximum_variance_m2'))
        self.required = int(p('minimum_report_frames'))
        self.consistency = float(p('report_consistency_m'))
        self.mission_timeout = float(p('mission_timeout_s'))
        self.detector_fault_grace = float(p('detector_fault_grace_s'))
        if (not all(math.isfinite(v) and v > 0 for v in
                    (self.age,self.stop_timeout,self.loc_timeout,self.variance,
                     self.consistency,self.mission_timeout)) or
                not math.isfinite(self.detector_fault_grace) or
                self.detector_fault_grace < 0 or self.required < 1):
            raise ValueError('Invalid mission limits')
        self.boot = str(uuid.uuid4())
        self.state, self.stop_token, self.stop_purpose = 'IDLE', '', ''
        self.config = {}
        self.runtime = {}
        self.runtime_receipt = self.adapter_receipt = 0.
        self.adapter_status = ''
        self.guard_boot = None
        self.deadline = None
        self.quiet_seen = False
        self.not_before = float('inf')
        self.last_report_stamp = 0.
        self.report_frames = 0
        self.report_points = []
        self.pending_report = None
        self.localization_reset_count = 0
        self.odom_healthy = False
        self.terminal_seen = False
        self.search_deadline = None
        self.detector_unready_since = None
        self.last_clock = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        realtime = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_pub = self.create_publisher(String, '/mission/state', latched)
        self.lease_pub = self.create_publisher(String, '/mission/lease', realtime)
        self.diagnostic_pub = self.create_publisher(String, '/mission/diagnostics', latched)
        self.localization_reset_pub = self.create_publisher(
            String, '/target/localization_reset', latched)
        self.report_pub = self.create_publisher(TargetEstimate, '/target/estimate', latched)
        self.point_pub = self.create_publisher(PointStamped, '/target_position', latched)
        self.found_pub = self.create_publisher(Bool, '/target_found', latched)
        self.create_subscription(String, '/target/configuration', self._configuration, latched)
        self.create_subscription(String, '/target/adapter_status', self._adapter, latched)
        self.create_subscription(
            String, '/search/runtime_state', self._runtime, realtime)
        self.create_subscription(
            TargetObservation, '/target/observation', self._observation, realtime)
        self.create_subscription(
            TargetEstimate, '/target/estimate_candidate', self._estimate, realtime)
        self.create_subscription(
            Odometry, str(p('odom_topic')), self._odom, realtime)
        self.mission_services = []
        for name, callback in [('start_mapping',self._start_mapping),
                               ('finish_mapping',self._finish_mapping),
                               ('start_search',self._start_search),
                               ('pause',self._pause),('resume',self._resume),
                               ('finish',self._finish),('reset',self._reset)]:
            self.mission_services.append(self.create_service(Trigger, '/mission/'+name, callback))
        self.create_timer(.1, self._tick)
        self.found_pub.publish(Bool(data=False))
        self._publish()

    def _now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def _publish(self):
        self.state_pub.publish(String(data=self.state))
        self.lease_pub.publish(String(data=json.dumps(dict(boot=self.boot,
            stamp=self._now(), state=self.state,
            camera_ready=(time.monotonic()-self.adapter_receipt <= self.age and
                          self.adapter_status in ('TARGET_CONFIGURED','TARGET_CONFIRMED')),
            episode=self.config.get('episode_id',''), stop_token=self.stop_token))))

    def _state(self, value):
        if value != self.state:
            self.get_logger().info(self.state+' -> '+value)
        self.state = value
        self._publish()

    def _configuration(self, msg):
        try:
            d = json.loads(msg.data)
            if d.get('episode_id') != self.config.get('episode_id'):
                if self.state in ('SEARCHING','STOPPING_FOR_TARGET','LOCALIZING_TARGET','PAUSED'):
                    self._stop('FAULT')
                self.config = d
                self.report_frames, self.last_report_stamp = 0, 0.
                self.found_pub.publish(Bool(data=False))
                self._publish()
        except (ValueError, TypeError):
            self._stop('FAULT')

    def _adapter(self, msg):
        self.adapter_status, self.adapter_receipt = msg.data, time.monotonic()

    def _runtime(self, msg):
        try:
            d = json.loads(msg.data)
            if d.get('mission_boot') != self.boot or not 0 <= self._now()-float(d['stamp']) <= self.age:
                return
            if self.guard_boot is not None and self.guard_boot != d['boot']:
                self._stop('FAULT')
            self.guard_boot = d['boot']
            self.runtime, self.runtime_receipt = d, time.monotonic()
        except (ValueError, TypeError, KeyError):
            pass

    def _runtime_fresh(self):
        return (bool(self.runtime) and time.monotonic()-self.runtime_receipt <= self.age and
                0 <= self._now()-float(self.runtime['stamp']) <= self.age)

    def _source_ready(self):
        return self._detector_ready() and self._runtime_ready()

    def _detector_ready(self):
        return (self.config.get('configured') and self.config.get('episode_id') and
                time.monotonic()-self.adapter_receipt <= self.age and
                self.adapter_status in ('TARGET_CONFIGURED','TARGET_CONFIRMED'))

    def _runtime_ready(self):
        return (self._runtime_fresh() and self.runtime.get('ready') and
                not self.runtime.get('operator_stop'))

    def _detector_fault_expired(self, steady):
        """Apply a bounded grace period only to detector freshness flicker."""
        if self._detector_ready():
            if self.detector_unready_since is not None:
                duration = steady-self.detector_unready_since
                self.get_logger().info(
                    f'Detector source recovered after {duration:.3f}s; continuing search')
            self.detector_unready_since = None
            return False
        if self.detector_unready_since is None:
            self.detector_unready_since = steady
            if self.detector_fault_grace > 0:
                self.get_logger().warning(
                    'Detector source temporarily unavailable; allowing '
                    f'{self.detector_fault_grace:.3f}s recovery grace')
        return steady-self.detector_unready_since >= self.detector_fault_grace

    def _odom(self, msg):
        q, p, v = msg.pose.pose.orientation, msg.pose.pose.position, msg.twist.twist
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if (msg.header.frame_id != self.odom_frame or not math.isfinite(norm) or
                abs(norm-1.) > 1e-3):
            self.gate.clear('ODOMETRY_FRAME_OR_QUATERNION_INVALID')
            self.odom_healthy = False
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        healthy, moved, why, metrics = self.gate.add(
            stamp,self._now(),time.monotonic(),msg.header.frame_id,
            p.x,p.y,yaw,v.linear.x,v.linear.y,v.angular.z)
        self.odom_healthy = healthy
        if moved and self.state == 'LOCALIZING_TARGET':
            self._invalidate_localization('ODOMETRY_MOTION_INTERRUPT', stamp,
                                          dict(reason=why, metrics=metrics))

    def _observation(self, msg):
        if self.state != 'SEARCHING':
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (msg.confirmed and msg.episode_id == self.config.get('episode_id') and
                msg.class_id == self.config.get('class_id') and
                msg.class_name == self.config.get('class_name') and
                math.isfinite(msg.confidence) and
                msg.confidence >= self.config.get('min_confidence',1.) and
                0 <= self._now()-stamp <= self.age):
            self._state('TARGET_DETECTED')
            self._begin_target_localization(stamp)

    def _begin_target_localization(self, observation_stamp):
        """Start localization immediately while navigation cancellation proceeds."""
        self.stop_token, self.stop_purpose = str(uuid.uuid4()), 'TARGET'
        self.deadline = time.monotonic()+self.stop_timeout
        self.quiet_seen = False
        self.terminal_seen = False
        self.not_before = observation_stamp
        self.report_frames, self.last_report_stamp = 0, 0.
        self.report_points = []
        self.pending_report = None
        self._publish_localization_reset('TARGET_CONFIRMED', observation_stamp)
        self._state('LOCALIZING_TARGET')

    def _publish_localization_reset(self, reason, not_before):
        self.localization_reset_count += 1
        payload = dict(episode_id=self.config.get('episode_id',''),
                       token=str(uuid.uuid4()), generation=self.localization_reset_count,
                       reason=reason, not_before=float(not_before), stamp=self._now())
        self.localization_reset_pub.publish(String(data=json.dumps(payload)))

    def _invalidate_localization(self, reason, not_before, detail=None):
        """Discard only the current fusion history; remain in localization."""
        self.not_before = max(self.not_before, float(not_before))
        self.report_frames, self.last_report_stamp = 0, 0.
        self.report_points = []
        self.pending_report = None
        self._publish_localization_reset(reason, self.not_before)
        self.diagnostic_pub.publish(String(data=json.dumps(dict(
            state=self.state, localization_interrupted=reason,
            detail=detail or {}, stop_token=self.stop_token))))

    def _stop(self, purpose):
        # Fresh nonce binds acknowledgement to this stop, not a latched old status.
        self.stop_token, self.stop_purpose = str(uuid.uuid4()), purpose
        self.deadline = time.monotonic()+self.stop_timeout
        self.quiet_seen = False
        self._state('STOPPING_FOR_TARGET' if purpose == 'TARGET' else 'STOPPING')

    def _quiet_ack(self):
        return (self._runtime_fresh() and self.runtime.get('stop_token') == self.stop_token and
                self.runtime.get('quiescent') and not self.runtime.get('unknown_terminal'))

    def _estimate(self, msg):
        if self.state != 'LOCALIZING_TARGET':
            return
        monitor_healthy = self.gate.evaluate(self._now(),time.monotonic())[0]
        if not (self.verified and self.camera_motion_verified and
                self.odom_healthy and monitor_healthy):
            self.report_frames, self.report_points = 0, []
            self.pending_report = None
            return
        if not estimate_is_reportable(msg,self.config.get('episode_id'),
            self.config.get('class_id'),self.config.get('class_name'),self._now(),
            self.not_before,self.age,self.variance):
            self.report_frames = 0
            self.report_points = []
            self.pending_report = None
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp <= self.last_report_stamp:
            return
        point = (msg.position.x,msg.position.y)
        if (stamp-self.last_report_stamp > self.age or any(
                math.hypot(point[0]-p[0],point[1]-p[1]) > self.consistency
                for p in self.report_points)):
            self.report_frames, self.report_points = 0, []
            self.pending_report = None
        self.last_report_stamp = stamp
        self.report_points.append(point)
        self.report_frames += 1
        # Localization may converge while cancellation is still in flight, but
        # final publication remains forbidden until Nav2 confirms a remote
        # terminal state tied to this stop token.
        if self.report_frames < self.required:
            return
        self.pending_report = msg
        if not self._quiet_ack():
            return
        self._publish_final(msg)

    def _publish_final(self, msg):
        monitor_healthy = self.gate.evaluate(self._now(),time.monotonic())[0]
        if not (self._quiet_ack() and self.verified and
                self.camera_motion_verified and self.odom_healthy and
                monitor_healthy and estimate_is_reportable(
                    msg,self.config.get('episode_id'),self.config.get('class_id'),
                    self.config.get('class_name'),self._now(),self.not_before,
                    self.age,self.variance)):
            return False
        msg.reason = 'REPORTED_AFTER_TERMINAL_WITH_MOTION_INTERRUPT_MONITOR'
        self.report_pub.publish(msg)
        point = PointStamped()
        point.header, point.point = msg.header, msg.position
        self.point_pub.publish(point)
        self.found_pub.publish(Bool(data=True))
        self.pending_report = None
        self._state('TARGET_REPORTED')
        return True

    def _tick(self):
        now, steady = self._now(), time.monotonic()
        if self.last_clock is not None and now < self.last_clock:
            self._stop('FAULT')
        self.last_clock = now
        self._publish()
        monitor_healthy, why, metrics = self.gate.evaluate(now,steady)
        self.odom_healthy = self.odom_healthy and monitor_healthy
        self.diagnostic_pub.publish(String(data=json.dumps(dict(state=self.state,
            motion_monitor_reason=why,motion_monitor=metrics,
            motion_interrupt_verified=self.verified,odom_healthy=self.odom_healthy,
            camera_motion_tf_verified=self.camera_motion_verified,
            runtime_fresh=self._runtime_fresh(),
            detector_ready=self._detector_ready(),
            detector_unready_s=(0. if self.detector_unready_since is None else
                                max(0.,steady-self.detector_unready_since)),
            stop_token=self.stop_token))))
        if self.state == 'SEARCHING':
            if self.runtime.get('operator_stop'):
                self._stop('OPERATOR')
            elif self.search_deadline is not None and steady >= self.search_deadline:
                self._stop('TIMEOUT')
            elif not self._runtime_ready():
                self._stop('FAULT')
            elif self._detector_fault_expired(steady):
                self._stop('FAULT')
            elif self.runtime.get('finished'):
                self._stop('SEARCH_COMPLETE')
        elif self.state in ('STOPPING','STOPPING_FOR_TARGET'):
            if steady >= self.deadline:
                self._state('STOP_UNVERIFIED')
            elif self._quiet_ack():
                self._state({'PAUSE':'PAUSED','FINISH':'FINISHED',
                             'SEARCH_COMPLETE':'SEARCH_EXHAUSTED',
                             'OPERATOR':'OPERATOR_STOPPED',
                             'TIMEOUT':'MISSION_TIMEOUT'}.get(self.stop_purpose,'FAULT'))
            else:
                self.quiet_seen = False
        elif self.state == 'LOCALIZING_TARGET':
            if self._quiet_ack() and not self.terminal_seen:
                self.terminal_seen = True
                self.deadline = steady+self.loc_timeout
                if self.pending_report is not None:
                    self._publish_final(self.pending_report)
            elif not self.terminal_seen and steady >= self.deadline:
                self._state('STOP_UNVERIFIED')
            elif self.terminal_seen and not self._quiet_ack():
                self._state('STOP_UNVERIFIED')
            elif self.terminal_seen and steady >= self.deadline:
                self._state('LOCALIZATION_FAILED')
        elif self.state == 'TARGET_REPORTED':
            self._state('FINISHED')
        if self.state != 'SEARCHING':
            self.detector_unready_since = None

    def _response(self, response, success, message):
        response.success, response.message = bool(success), message
        return response

    def _start_mapping(self, request, response):
        if self.state not in ('IDLE','WAIT_FOR_TARGET'):
            return self._response(response,False,'Invalid state')
        self._state('MAPPING')
        return self._response(response,True,'Mapping phase only; external SLAM/exploration managed separately')

    def _finish_mapping(self, request, response):
        if self.state != 'MAPPING':
            return self._response(response,False,'Not MAPPING')
        self._state('WAIT_FOR_TARGET')
        return self._response(response,True,'Operator acknowledges saved map/localization; configure target next')

    def _start_search(self, request, response):
        if (self.state != 'WAIT_FOR_TARGET' or not self._source_ready() or
                not self.runtime.get('quiescent') or not self.verified or
                not self.odom_healthy or self.runtime.get('finished')):
            return self._response(response,False,'Require configured fresh detector, robot readiness, verified motion interrupt monitor and WAIT_FOR_TARGET')
        self.stop_token = ''
        self.search_deadline = time.monotonic()+self.mission_timeout
        self.detector_unready_since = None
        self._state('SEARCHING')
        return self._response(response,True,'Search started with motion-interrupt monitoring; no extra motion authorization granted')

    def _pause(self, request, response):
        if self.state != 'SEARCHING':
            return self._response(response,False,'Pause only during SEARCHING')
        self._stop('PAUSE')
        return self._response(response,True,'Pause requested; wait for PAUSED, not service success')

    def _resume(self, request, response):
        if self.state != 'PAUSED' or not self._source_ready() or not self._quiet_ack():
            return self._response(response,False,'Resume refused; require fresh inputs and verified pause')
        self.stop_token = ''
        self._state('SEARCHING')
        return self._response(response,True,'Resumed; navigation budget unchanged')

    def _finish(self, request, response):
        if self.state not in ('SEARCHING','PAUSED'):
            return self._response(response,False,'Finish only during search/pause')
        self._stop('FINISH')
        return self._response(response,True,'Finish requested; await terminal/window verification')

    def _reset(self, request, response):
        if self.state not in ('IDLE','FINISHED','SEARCH_EXHAUSTED','LOCALIZATION_FAILED',
                              'PAUSED','OPERATOR_STOPPED','MISSION_TIMEOUT','FAULT') or not self._quiet_ack():
            return self._response(response,False,'Reset refused while active or stop unverified')
        self.config = {}
        self.found_pub.publish(Bool(data=False))
        self._state('IDLE')
        return self._response(response,True,'Mission reset; reconfigure target; memory/budget rearm is separate')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetMissionCoordinator()
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
