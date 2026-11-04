import ast
from pathlib import Path
import unittest
from ee3033_search_policy.yolo_adapter_session import AdapterSession

path = Path(__file__).resolve().parents[1] / 'ee3033_search_policy' / 'continuous_observation_adapter.py'
tree = ast.parse(path.read_text())
cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == 'ObservationSession')
env = {'AdapterSession':AdapterSession}
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),env)
ObservationSession = env['ObservationSession']


class TestContinuousObservationSession(unittest.TestCase):
    def session(self):
        s=ObservationSession({'39':'bottle'})
        s.set_state('WAIT_FOR_TARGET')
        s._accept_source(10.,'optical',10.,10.)
        s.configure('bottle',.65,10.,10.)
        s.set_state('SEARCHING')
        return s

    def test_confirmed_stream_continues_in_post_detection_states(self):
        for state in ('STOPPING_FOR_TARGET','LOCALIZING_TARGET'):
            s=self.session()
            s.frames.confirmed=True
            s.set_state(state)
            self.assertTrue(s._accept_source(10.1,'optical',10.1,10.1))
            self.assertEqual(s.state,state)

    def test_no_first_confirmation_outside_search(self):
        s=self.session()
        s.set_state('LOCALIZING_TARGET')
        self.assertFalse(s._accept_source(10.1,'optical',10.1,10.1))

    def test_post_detection_stale_and_repeat_still_rejected(self):
        s=self.session()
        s.frames.confirmed=True
        s.set_state('LOCALIZING_TARGET')
        self.assertFalse(s._accept_source(10.,'optical',10.1,10.1))
        self.assertFalse(s._accept_source(10.1,'optical',11.,11.))

    def test_idle_clears_episode_and_post_detection_permission(self):
        s=self.session()
        s.frames.confirmed=True
        s.set_state('IDLE')
        s.set_state('LOCALIZING_TARGET')
        self.assertFalse(s._accept_source(10.1,'optical',10.1,10.1))


if __name__ == '__main__':
    unittest.main()
