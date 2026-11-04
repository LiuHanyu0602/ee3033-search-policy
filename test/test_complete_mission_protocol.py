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
        m._runtime_fresh=lambda: True
        m._source_ready=lambda: True
        m.runtime={}
        m.last_clock=None
        m.quiet_seen=False
        m.diagnostic_pub=Mock()
        m.report_pub=Mock()
        m.point_pub=Mock()
        m.found_pub=Mock()
        m.boot='m'
        m.search_deadline=None
        m.stop_token=''
        m.variance=.25
        m.consistency=.1
        m.required=3
        m.report_frames=0
        m.report_points=[]
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
        self.assertEqual(m.state,'STOPPING_FOR_TARGET')
        self.assertTrue(m.stop_token)
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
        self.assertEqual(m.state,'STOPPING_FOR_TARGET')

    def ready_to_localize(self):
        m=self.mission()
        m._observation(self.observation())
        m.runtime=dict(stop_token=m.stop_token,quiescent=True,unknown_terminal=False)
        m._tick()  # Clear window only after matching terminal acknowledgement.
        self.assertTrue(m.quiet_seen)
        m._tick()
        self.assertEqual(m.state,'LOCALIZING_TARGET')
        return m

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

    def test_unverified_limits_never_advance_to_localization(self):
        m=self.mission()
        m._observation(self.observation())
        m.runtime=dict(stop_token=m.stop_token,quiescent=True,unknown_terminal=False)
        m.verified=False
        m._tick()
        m._tick()
        self.assertEqual(m.state,'STOPPING_FOR_TARGET')

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

    def test_resume_does_not_create_new_time_budget(self):
        m=self.mission()
        m.state='PAUSED'
        m._quiet_ack=lambda: True
        m.search_deadline=1234.
        response=m._resume(None,NS())
        self.assertTrue(response.success)
        self.assertEqual(m.search_deadline,1234.)

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


if __name__ == '__main__':
    unittest.main()
