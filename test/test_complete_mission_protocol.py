"""ROS-free method tests. Real DDS/Action integration is separate acceptance."""
import ast
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import uuid
from ee3033_search_policy.balance_observation_gate import (
    estimate_is_reportable, navigation_terminal_known,
)

ROOT = Path(__file__).resolve().parents[1] / 'ee3033_search_policy'


class Base:
    def _choose(self):
        self.base_choose = True

    def _nav_done(self, index, future):
        self.base_done = True

    def _finish_observation(self):
        self.base_observation = True


def load_class(filename, name, base):
    tree = ast.parse((ROOT / filename).read_text())
    cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == name)
    env = dict(json=json,time=time,uuid=uuid,math=math,
               String=lambda **kw: NS(**kw), Bool=lambda **kw: NS(**kw),
               PointStamped=lambda: NS(),Node=base,SearchManager=base,
               estimate_is_reportable=estimate_is_reportable,
               navigation_terminal_known=navigation_terminal_known)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),filename,'exec'),env)
    return env[name]


Mission = load_class('target_mission_coordinator.py','TargetMissionCoordinator',Base)
Guard = load_class('guarded_search_manager.py','GuardedSearchManager',Base)


class TestCompleteMissionProtocol(unittest.TestCase):
    def mission(self):
        m = Mission.__new__(Mission)
        m.state='SEARCHING'
        m.config=dict(episode_id='e',class_id=39,class_name='bottle',min_confidence=.65,configured=True)
        m._now=lambda: 100.
        m.get_logger=lambda: Mock()
        m._publish=Mock()
        m.stop_timeout=10.
        m.loc_timeout=10.
        m.age=.5
        m.gate=Mock()
        m.gate.evaluate.return_value=(True,'BOUNDED',{})
        m.verified=m.camera_motion_verified=True
        m.odom_healthy=True
        m.terminal_seen=False
        m._runtime_fresh=lambda: True
        m._runtime_ready=lambda: True
        m._detector_ready=lambda: True
        m._source_ready=lambda: True
        m.runtime={}
        m.last_clock=None
        m.quiet_seen=False
        m.diagnostic_pub=Mock()
        m.localization_reset_pub=Mock()
        m.localization_reset_count=0
        m.report_pub=Mock()
        m.point_pub=Mock()
        m.found_pub=Mock()
        m.boot='m'
        m.search_deadline=None
        m.detector_fault_grace=.75
        m.detector_unready_since=None
        m.stop_token=''
        m.variance=.25
        m.consistency=.1
        m.required=3
        m.report_frames=0
        m.report_points=[]
        m.pending_report=None
        m.last_report_stamp=0.
        return m

    def observation(self, episode='e'):
        return NS(confirmed=True,episode_id=episode,class_id=39,class_name='bottle',
                  confidence=.9,header=NS(stamp=NS(sec=100,nanosec=0)))

    def estimate(self, ns=0):
        return NS(valid=True,header=NS(frame_id='map',stamp=NS(sec=100,nanosec=ns)),
                  episode_id='e',class_id=39,class_name='bottle',
                  position=NS(x=1.,y=2.,z=0.),covariance_xy=[.01,0.,0.,.01],
                  quality=.8,method='ground',reason='candidate')

    def test_detection_inhibits_before_report(self):
        m=self.mission()
        m._observation(self.observation())
        self.assertEqual(m.state,'LOCALIZING_TARGET')
        self.assertTrue(m.stop_token)
        m.localization_reset_pub.publish.assert_called_once()
        m.report_pub.publish.assert_not_called()

    def test_wrong_episode_cannot_stop_search(self):
        m=self.mission()
        m._observation(self.observation('old'))
        self.assertEqual(m.state,'SEARCHING')

    def test_old_stop_ack_cannot_advance(self):
        m=self.mission()
        m._observation(self.observation())
        m.runtime=dict(stop_token='previous',quiescent=True,unknown_terminal=False)
        m._tick()
        self.assertEqual(m.state,'LOCALIZING_TARGET')
        self.assertFalse(m.terminal_seen)

    def ready_to_localize(self):
        m=self.mission()
        m._observation(self.observation())
        self.assertEqual(m.state,'LOCALIZING_TARGET')
        m.runtime=dict(stop_token=m.stop_token,quiescent=True,unknown_terminal=False)
        m._tick()
        self.assertTrue(m.terminal_seen)
        return m

    def test_localization_accumulates_before_terminal_but_does_not_publish(self):
        m=self.mission()
        m._observation(self.observation())
        m.runtime=dict(stop_token='old',quiescent=True,unknown_terminal=False)
        m._now=lambda: 100.3
        for ns in (100_000_000,200_000_000,300_000_000):
            m._estimate(self.estimate(ns))
        self.assertEqual(m.report_frames,3)
        m.report_pub.publish.assert_not_called()
        m.runtime=dict(stop_token=m.stop_token,quiescent=True,unknown_terminal=False)
        m._tick()
        m.report_pub.publish.assert_called_once()
        self.assertEqual(m.state,'TARGET_REPORTED')

    def test_full_event_stop_localize_report_finish(self):
        m=self.ready_to_localize()
        m._now=lambda: 100.3
        for ns in (100_000_000,200_000_000,300_000_000):
            m._estimate(self.estimate(ns))
        self.assertEqual(m.state,'TARGET_REPORTED')
        m.report_pub.publish.assert_called_once()
        m.point_pub.publish.assert_called_once()
        m._tick()
        self.assertEqual(m.state,'FINISHED')

    def test_unverified_motion_monitor_blocks_final_report(self):
        m=self.mission()
        m._observation(self.observation())
        m.runtime=dict(stop_token=m.stop_token,quiescent=True,unknown_terminal=False)
        m.verified=False
        m._tick()
        m._estimate(self.estimate())
        self.assertEqual(m.state,'LOCALIZING_TARGET')
        m.report_pub.publish.assert_not_called()

    def test_unknown_terminal_times_out_not_success(self):
        m=self.mission()
        m._observation(self.observation())
        m.runtime=dict(stop_token=m.stop_token,quiescent=True,unknown_terminal=True)
        m.deadline=0.
        m._tick()
        self.assertEqual(m.state,'STOP_UNVERIFIED')
        m.report_pub.publish.assert_not_called()

    def test_missing_dynamic_pitch_verification_blocks_final_report(self):
        m=self.ready_to_localize()
        m.camera_motion_verified=False
        m._estimate(self.estimate())
        m.report_pub.publish.assert_not_called()

    def test_duplicate_estimate_does_not_count_as_new_frame(self):
        m=self.ready_to_localize()
        for _ in range(5):
            m._estimate(self.estimate())
        self.assertEqual(m.report_frames,1)
        m.report_pub.publish.assert_not_called()

    def test_inconsistent_coordinates_reset_sequence(self):
        m=self.ready_to_localize()
        m._now=lambda: 100.2
        m._estimate(self.estimate())
        other=self.estimate(100_000_000)
        other.position.x=2.
        m._estimate(other)
        self.assertEqual(m.report_frames,1)
        m.report_pub.publish.assert_not_called()

    def test_large_motion_clears_fusion_without_aborting_localization(self):
        m=self.mission()
        m._observation(self.observation())
        m.report_frames=2
        m.report_points=[(1.,2.)]
        m._invalidate_localization('ODOMETRY_MOTION_INTERRUPT',100.2)
        self.assertEqual(m.state,'LOCALIZING_TARGET')
        self.assertEqual(m.report_frames,0)
        self.assertEqual(m.report_points,[])
        self.assertEqual(m.not_before,100.2)
        self.assertEqual(m.localization_reset_pub.publish.call_count,2)

    def test_resume_does_not_create_new_time_budget(self):
        m=self.mission()
        m.state='PAUSED'
        m._quiet_ack=lambda: True
        m.search_deadline=1234.
        response=m._resume(None,NS())
        self.assertTrue(response.success)
        self.assertEqual(m.search_deadline,1234.)

    def test_start_search_waits_for_runtime_rearm(self):
        m=self.mission()
        m.state='WAIT_FOR_TARGET'
        m.runtime=dict(quiescent=True,finished=True)
        response=m._start_search(None,NS())
        self.assertFalse(response.success)
        self.assertEqual(m.state,'WAIT_FOR_TARGET')

    def test_transient_detector_stall_uses_bounded_grace(self):
        m=self.mission()
        ready=[False]
        m._detector_ready=lambda: ready[0]
        self.assertFalse(m._detector_fault_expired(10.0))
        self.assertFalse(m._detector_fault_expired(10.5))
        ready[0]=True
        self.assertFalse(m._detector_fault_expired(10.6))
        self.assertIsNone(m.detector_unready_since)

    def test_sustained_detector_stall_expires_grace(self):
        m=self.mission()
        m._detector_ready=lambda: False
        self.assertFalse(m._detector_fault_expired(10.0))
        self.assertTrue(m._detector_fault_expired(10.8))

    def test_runtime_fault_remains_fail_fast(self):
        m=self.mission()
        m._runtime_ready=lambda: False
        m._tick()
        self.assertEqual(m.state,'STOPPING')
        self.assertEqual(m.stop_purpose,'FAULT')

    def test_reset_refused_while_unverified(self):
        m=self.mission()
        m.state='STOP_UNVERIFIED'
        m._quiet_ack=lambda: False
        self.assertFalse(m._reset(None,NS()).success)

    def guard(self):
        g=Guard.__new__(Guard)
        g.mission_state='SEARCHING'
        g.lease={'episode':'e'}
        g.operator_cancel_requested=False
        g.finished=False
        g._healthy=lambda: (True,'READY')
        g.busy=True
        return g

    def test_late_planning_result_cannot_dispatch_after_stop(self):
        for state in ('STOPPING_FOR_TARGET','PAUSED','IDLE','LOCALIZING_TARGET'):
            g=self.guard()
            g.mission_state=state
            g._choose()
            self.assertFalse(getattr(g,'base_choose',False))

    def test_lease_or_operator_inhibit_blocks_dispatch(self):
        g=self.guard()
        g._healthy=lambda: (False,'LEASE_STALE')
        g._choose()
        self.assertFalse(getattr(g,'base_choose',False))
        g=self.guard()
        g.operator_cancel_requested=True
        g._choose()
        self.assertFalse(getattr(g,'base_choose',False))

    def test_local_timeout_latches_unknown_before_base_callback(self):
        g=self.guard()
        g._status=Mock()
        result=NS(accepted=True,status=5,timed_out=True)
        g._nav_done(0,NS(result=lambda: result))
        self.assertTrue(g.unknown_terminal)
        self.assertTrue(g.finished)
        self.assertFalse(getattr(g,'base_done',False))

    def test_known_terminal_during_stop_cannot_start_observation(self):
        g=self.guard()
        g.mission_state='STOPPING_FOR_TARGET'
        result=NS(accepted=True,status=5,timed_out=False)
        g._nav_done(0,NS(result=lambda: result))
        self.assertIsNone(g.active_navigation)
        self.assertIsNone(g.observation_deadline)
        self.assertFalse(getattr(g,'base_done',False))

    def observation_guard(self, yaw):
        g=self.guard()
        g._robot_pose=lambda: (0.,0.,yaw)
        g.selected_index=0
        g.lease={'episode':'e','camera_ready':True}
        g.candidates=NS(world_x=[0.1],world_y=[0.],headings=[0.])
        g.observation_position_tolerance=.15
        g.observation_yaw_tolerance=.25
        g.observation_deadline=1.
        g.navigation_goal_count=1
        g.max_navigation_goals=1
        g.single_goal_mode=True
        g._failure=Mock()
        g._finish_search=Mock()
        return g

    def test_observation_accepts_deployed_nav2_yaw_tolerance(self):
        g=self.observation_guard(.24)
        g._finish_observation()
        self.assertTrue(g.base_observation)
        g._failure.assert_not_called()

    def test_observation_failure_reports_pose_metrics(self):
        g=self.observation_guard(.26)
        g._finish_observation()
        message=g._failure.call_args.args[1]
        self.assertIn('distance_m=0.100',message)
        self.assertIn('yaw_error_rad=0.260',message)
        self.assertIn('yaw_limit_rad=0.250',message)
        g._finish_search.assert_called_once_with(
            'SINGLE_GOAL_OBSERVATION_UNVERIFIED')

    def lease_guard(self, episode='old'):
        g=Guard.__new__(Guard)
        g.get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
        g.lease_timeout=.6
        g.lease=dict(boot='mission',stamp=100.,state='WAIT_FOR_TARGET',
                     episode=episode)
        g.lease_receipt=0.
        g.mission_state='WAIT_FOR_TARGET'
        g.unknown_terminal=False
        g.decision_serial=0
        g.active_plan=None
        g.memory=Mock()
        g.observation_deadline=None
        g._cancel=Mock()
        g.navigation_goal_count=1
        g.operator_cancel_requested=True
        g.finished=g.busy=True
        return g

    def test_new_episode_rearms_while_waiting_for_target(self):
        g=self.lease_guard()
        msg=NS(data=json.dumps(dict(
            boot='mission',stamp=100.,state='WAIT_FOR_TARGET',episode='new')))
        g._lease_cb(msg)
        self.assertEqual(g.navigation_goal_count,0)
        self.assertFalse(g.operator_cancel_requested)
        self.assertFalse(g.finished)
        self.assertFalse(g.busy)

    def test_same_episode_does_not_rearm_operator_stop(self):
        g=self.lease_guard()
        msg=NS(data=json.dumps(dict(
            boot='mission',stamp=100.,state='WAIT_FOR_TARGET',episode='old')))
        g._lease_cb(msg)
        self.assertEqual(g.navigation_goal_count,1)
        self.assertTrue(g.operator_cancel_requested)
        self.assertTrue(g.finished)


if __name__ == '__main__':
    unittest.main()
