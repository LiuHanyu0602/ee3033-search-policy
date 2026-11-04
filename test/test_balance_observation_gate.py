import math
from types import SimpleNamespace as NS
import unittest
from ee3033_search_policy.balance_observation_gate import (
    BalanceObservationGate, StabilityLimits, estimate_is_reportable,
    navigation_terminal_known,
)


class TestBalanceObservationGate(unittest.TestCase):
    def fill(self, mode='jitter'):
        gate = BalanceObservationGate()
        for i in range(81):
            t = 10+i*.025
            v = .025*math.sin(i)
            x = .008*math.sin(i)
            yaw, w = .01*math.sin(i), .04*math.sin(i)
            if mode == 'translation':
                x, v = i*.004, .16
            if mode == 'rotation':
                yaw, w = i*.01, .4
            if mode == 'fast_no_drift':
                v = .15*(-1)**i
            if mode == 'wrap':
                yaw = math.atan2(math.sin(math.pi+.01*math.sin(i)),
                                 math.cos(math.pi+.01*math.sin(i)))
            gate.add(t,t,t,'odom_combined',x,0.,yaw,v,0.,w)
        return gate

    def test_balancing_does_not_require_zero_velocity(self):
        ok, why, metrics = self.fill().evaluate(12.,12.)
        self.assertTrue(ok, why)
        self.assertGreater(metrics['linear_rms_mps'],0.)

    def test_translation_rejected(self):
        self.assertFalse(self.fill('translation').evaluate(12.,12.)[0])

    def test_rotation_rejected(self):
        self.assertFalse(self.fill('rotation').evaluate(12.,12.)[0])

    def test_opposite_velocities_cannot_average_to_stationary(self):
        self.assertFalse(self.fill('fast_no_drift').evaluate(12.,12.)[0])

    def test_yaw_wrap_not_false_large_rotation(self):
        self.assertTrue(self.fill('wrap').evaluate(12.,12.)[0])

    def test_stale_receipt_or_source_rejected(self):
        g = self.fill()
        self.assertFalse(g.evaluate(12.5,12.)[0])
        self.assertFalse(g.evaluate(12.,12.5)[0])

    def test_repeated_stamp_resets_window(self):
        g = self.fill()
        self.assertFalse(g.add(12.,12.,12.01,'odom_combined',0,0,0,0,0,0))
        self.assertFalse(g.evaluate(12.,12.01)[0])

    def test_gap_and_frame_change_restart_window(self):
        for frame, stamp in [('different_odom',12.025),('odom_combined',13.)]:
            g = self.fill()
            g.add(stamp,stamp,stamp,frame,0,0,0,0,0,0)
            self.assertFalse(g.evaluate(stamp,stamp)[0])

    def test_nonfinite_and_future_rejected(self):
        for stamp, x in [(13.,0.),(12.1,float('nan'))]:
            g = self.fill()
            self.assertFalse(g.add(stamp,12.1,12.1,'odom_combined',x,0,0,0,0,0))

    def test_limits_validation(self):
        for kwargs in [dict(window_s=0),dict(max_age_s=float('nan')),dict(min_samples=1)]:
            with self.assertRaises(ValueError):
                StabilityLimits(**kwargs)

    def test_remote_terminal_not_local_timeout(self):
        for accepted, status, timeout, expected in [
            (True,4,False,True),(True,5,False,True),(True,6,False,True),
            (False,0,False,True),(None,0,False,False),(True,2,False,False),
            (True,5,True,False)]:
            self.assertEqual(navigation_terminal_known(NS(
                accepted=accepted,status=status,timed_out=timeout)),expected)

    def estimate(self):
        return NS(valid=True,header=NS(frame_id='map',stamp=NS(sec=100,nanosec=0)),
                  episode_id='e',class_id=39,class_name='bottle',
                  position=NS(x=1.,y=2.,z=0.),covariance_xy=[.01,0.,0.,.01],
                  quality=.8,method='ground')

    def test_report_boundary(self):
        m = self.estimate()
        check = lambda: estimate_is_reportable(m,'e',39,'bottle',100.1,99.9)
        self.assertTrue(check())
        for field,value in [('episode_id','old'),('valid',False),
                            ('covariance_xy',[.01,.1,.1,.01]),
                            ('quality',float('nan'))]:
            m = self.estimate()
            setattr(m,field,value)
            self.assertFalse(check())
        self.assertFalse(estimate_is_reportable(self.estimate(),'e',39,'bottle',101.,99.9))
        self.assertFalse(estimate_is_reportable(self.estimate(),'e',39,'bottle',100.1,100.01))


if __name__ == '__main__':
    unittest.main()
