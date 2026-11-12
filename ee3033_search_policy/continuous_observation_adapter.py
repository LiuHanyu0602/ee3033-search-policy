"""Keep fresh confirmed observations flowing while stopping/localizing."""
import rclpy
from .yolo_adapter_session import AdapterSession
from .yolo_detection_adapter_node import YoloDetectionAdapter


class ObservationSession(AdapterSession):
    def _accept_source(self, stamp, frame_id, ros_now, steady_now):
        state = self.state
        # No first confirmation outside SEARCHING; no source timestamp rewriting.
        if self.frames.confirmed and state in (
                'STOPPING_FOR_TARGET', 'ALIGNING_TARGET', 'LOCALIZING_TARGET'):
            self.state = 'SEARCHING'
            try:
                return super()._accept_source(stamp, frame_id, ros_now, steady_now)
            finally:
                self.state = state
        return super()._accept_source(stamp, frame_id, ros_now, steady_now)


class ContinuousObservationAdapter(YoloDetectionAdapter):
    def __init__(self):
        super().__init__()
        p = lambda k: self.get_parameter(k).value
        self.session = ObservationSession(self.session.target.labels,
            required_frames=int(p('required_frames')), max_age_s=float(p('max_age_s')),
            max_gap_s=float(p('max_gap_s')), minimum_iou=float(p('minimum_iou')))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ContinuousObservationAdapter()
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
