"""ROS-independent lifecycle for a configured YOLO search episode."""
import uuid

from .yolo_detection_core import FrameConfirmation, TargetConfiguration, select_observation


class AdapterSession:
    def __init__(self, labels, **confirmation_options):
        self.target = TargetConfiguration(labels)
        self.frames = FrameConfirmation(**confirmation_options)
        self.state = 'UNKNOWN'
        self.episode_id = ''
        self.last_source_stamp = None
        self.last_receipt = None
        self.last_clock = None

    def set_state(self, state):
        if state == self.state:
            return
        self.state = state
        if state in ('IDLE', 'MAPPING'):
            self.clear()
        elif state != 'SEARCHING' and not self.frames.confirmed:
            self.frames.reset()

    def clear(self):
        self.target.reset('IDLE', False)
        self.frames.reset()
        self.episode_id = ''
        self.last_source_stamp = self.last_receipt = None

    def tick(self, ros_now, steady_now):
        if self.last_clock is not None and ros_now < self.last_clock:
            self.clear()
        self.last_clock = ros_now
        self.frames.tick(ros_now)
        return self.ready(ros_now, steady_now)

    def ready(self, ros_now, steady_now):
        return (self.last_source_stamp is not None and self.last_receipt is not None and
                0 <= ros_now-self.last_source_stamp <= self.frames.max_age and
                0 <= steady_now-self.last_receipt <= self.frames.max_age)

    def configure(self, name, threshold, ros_now, steady_now):
        self.target.configure(name, threshold, self.state, self.ready(ros_now, steady_now))
        self.frames.reset()
        self.episode_id = str(uuid.uuid4())
        return self.episode_id

    def ingest(self, msg, width, height, ros_now, steady_now):
        self.tick(ros_now, steady_now)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if (stamp <= 0 or not msg.header.frame_id or
                not 0 <= ros_now-stamp <= self.frames.max_age):
            if not self.frames.confirmed:
                self.frames.update(None, ros_now)
            return None
        if self.last_source_stamp is not None and stamp <= self.last_source_stamp:
            return None
        self.last_source_stamp, self.last_receipt = stamp, steady_now
        if self.target.class_id is None or self.state != 'SEARCHING':
            return None
        observation = select_observation(msg, self.target, width, height)
        self.frames.update(observation, ros_now)
        return observation
