"""YOLO adapter for typed detections or deployed NCNN JSON, without navigation.

This phase does NOT publish /target_found; consumers must be integrated later.
Run from repo root: python3 -m ee3033_search_policy.yolo_detection_adapter_node.
"""
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from ee3033_search_policy_interfaces.msg import TargetObservation
from ee3033_search_policy_interfaces.srv import SetTargetClass

from .yolo_adapter_session import AdapterSession
from .yolo_bearing import RawCameraCalibration
from .yolo_json_input import NcnnJsonInput


class YoloDetectionAdapter(Node):
    def __init__(self):
        super().__init__('yolo_detection_adapter')
        calibration = Path(__file__).resolve().parents[1] / 'results' / 'updated_calibration' / 'calibration.npz'
        if not calibration.is_file():
            calibration = Path(get_package_share_directory('ee3033_search_policy')) / 'calibration' / 'calibration.npz'
        defaults = {
            'labels_file': '', 'detections_topic': '/yolo/detections',
            'camera_info_topic': '/camera_info', 'mission_topic': '/mission/state',
            'required_frames': 3, 'max_age_s': .5, 'max_gap_s': .4,
            'minimum_iou': .2, 'geometry_verified': False,
            'calibration_file': str(calibration),
            'input_format': 'detection_array', 'image_topic': '/camera/image_raw',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        input_format = str(p('input_format'))
        if input_format not in ('detection_array', 'ncnn_json'):
            raise ValueError('input_format must be detection_array or ncnn_json')
        self.json_input = NcnnJsonInput()
        self.calibration = RawCameraCalibration.load(
            Path(str(p('calibration_file'))).expanduser())
        # Missing labels fail at startup: never guess class IDs or download models.
        with Path(str(p('labels_file'))).expanduser().open(encoding='utf-8') as f:
            labels = json.load(f)
        if not isinstance(labels, dict):
            raise ValueError('labels_file must contain {class_id: class_name}')
        self.session = AdapterSession(labels, required_frames=int(p('required_frames')),
            max_age_s=float(p('max_age_s')), max_gap_s=float(p('max_gap_s')),
            minimum_iou=float(p('minimum_iou')))
        # Operator must verify CameraInfo describes detector pixels (raw/rectified).
        self.geometry_verified = bool(p('geometry_verified'))
        self.camera = None
        self.last_frame = ''
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensors = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.observations = self.create_publisher(TargetObservation, '/target/observation', 10)
        self.status_pub = self.create_publisher(String, '/target/adapter_status', latched)
        self.config_pub = self.create_publisher(String, '/target/configuration', latched)
        self.classes_pub = self.create_publisher(String, '/target/available_classes', latched)
        self.classes_pub.publish(String(data=json.dumps(self.session.target.labels)))
        if input_format == 'ncnn_json':
            detections_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(String, str(p('detections_topic')), self._detect_json,
                                     detections_qos)
            self.create_subscription(Image, str(p('image_topic')), self._image, sensors)
        else:
            from yolo_msgs.msg import DetectionArray
            self.create_subscription(DetectionArray, str(p('detections_topic')),
                                     self._detect, sensors)
        self.create_subscription(CameraInfo, str(p('camera_info_topic')), self._camera, sensors)
        self.create_subscription(String, str(p('mission_topic')), self._state, latched)
        self.create_service(SetTargetClass, '/target/set_class', self._configure)
        self.create_service(Trigger, '/target/reset', self._reset)
        self.create_timer(.1, self._tick)
        self._configuration()
        self.get_logger().warning('YOLO adapter only: no /target_found or navigation output')

    def _now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def _camera(self, msg):
        try:
            self.calibration.validate_camera_info(msg)
            if self.camera is not None and msg.header.frame_id != self.camera.header.frame_id:
                self.session.clear()
                self.json_input.clear()
                self.last_frame = ''
                self._configuration()
            self.camera = msg
            self.json_input.record_camera_info(msg, time.monotonic())
        except ValueError as exc:
            self.camera = None
            self.last_frame = ''
            self.json_input.clear()
            self.session.clear()
            self._configuration()
            self.get_logger().error('Rejected CameraInfo: ' + str(exc),
                                    throttle_duration_sec=2.0)
            return

    def _state(self, msg):
        old_episode = self.session.episode_id
        self.session.set_state(msg.data.strip().upper())
        if old_episode != self.session.episode_id:
            self._configuration()

    def _configuration(self):
        t = self.session.target
        self.config_pub.publish(String(data=json.dumps({
            'episode_id': self.session.episode_id, 'configured': t.class_id is not None,
            'class_id': t.class_id, 'class_name': t.class_name,
            'min_confidence': t.threshold})))

    def _configure(self, request, response):
        response.class_id = -1
        try:
            if not self.geometry_verified or self.camera is None or self.last_frame != self.camera.header.frame_id:
                raise ValueError('Verify detector pixel geometry and matching optical CameraInfo frame first')
            self.session.configure(request.class_name, request.min_confidence,
                                   self._now(), time.monotonic())
            response.success = True
            response.class_id = self.session.target.class_id
            response.normalized_class_name = self.session.target.class_name
            response.episode_id = self.session.episode_id
            response.message = 'Target locked; start mission separately'
            self._configuration()
        except (ValueError, TypeError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _reset(self, request, response):
        if self.session.state not in ('IDLE', 'WAIT_FOR_TARGET', 'FINISHED'):
            response.success, response.message = False, 'Reset refused in active/unknown mission state'
        else:
            self.session.clear()
            self._configuration()
            response.success, response.message = True, 'Adapter episode cleared; navigation not affected'
        return response

    def _detect(self, msg):
        if not self.geometry_verified or self.camera is None:
            return
        if msg.header.frame_id != self.camera.header.frame_id:
            return
        self.last_frame = msg.header.frame_id
        try:
            o = self.session.ingest(msg, self.camera.width, self.camera.height,
                                    self._now(), time.monotonic())
            self._publish_observation(o, msg.header)
        except (ValueError, TypeError, AttributeError, OverflowError) as exc:
            self.session.frames.update(None, self._now())
            self.get_logger().warning('Rejected detection: ' + str(exc))

    def _image(self, msg):
        if not self.geometry_verified or self.camera is None:
            return
        try:
            if ((msg.width, msg.height, msg.header.frame_id) !=
                    (self.camera.width, self.camera.height, self.camera.header.frame_id)):
                raise ValueError('Source Image size/frame does not match calibrated CameraInfo')
            self.json_input.record_image(msg, time.monotonic())
        except ValueError as exc:
            self.json_input.clear()
            self.last_frame = ''
            self.session.clear()
            self._configuration()
            self.get_logger().warning('Rejected source Image: ' + str(exc),
                                      throttle_duration_sec=2.0)

    def _detect_json(self, msg):
        if not self.geometry_verified or self.camera is None:
            return
        try:
            now, steady_now = self._now(), time.monotonic()
            frame, image = self.json_input.match(
                msg.data, now, steady_now, self.session.frames.max_age)
            self.last_frame = frame.frame_id
            o = self.session.ingest_frame(frame, image.width, image.height, now, steady_now)
            header = Header()
            header.stamp.sec, header.stamp.nanosec = divmod(image.stamp_ns, 1_000_000_000)
            header.frame_id = image.frame_id
            self._publish_observation(o, header)
        except (ValueError, TypeError, AttributeError, OverflowError) as exc:
            self.session.frames.update(None, self._now())
            self.get_logger().warning('Rejected NCNN JSON: ' + str(exc),
                                      throttle_duration_sec=2.0)

    def _publish_observation(self, o, header):
        if o is None:
            return
        try:
            bearing = self.calibration.bearing(o.cx, o.cy)
        except ValueError:
            self.session.frames.reset()
            raise
        result = TargetObservation()
        result.header = header
        result.episode_id = self.session.episode_id
        result.class_id, result.class_name = o.class_id, o.class_name
        result.confidence, result.tracking_id = o.confidence, o.tracking_id
        result.center_x, result.center_y = o.cx, o.cy
        result.width, result.height = o.width, o.height
        result.image_width, result.image_height = self.camera.width, self.camera.height
        result.bearing_rad = bearing
        result.confirmed = self.session.frames.confirmed
        self.observations.publish(result)

    def _tick(self):
        old_episode = self.session.episode_id
        ready = self.session.tick(self._now(), time.monotonic())
        if old_episode != self.session.episode_id:
            self._configuration()
        status = ('GEOMETRY_UNVERIFIED' if not self.geometry_verified else
                  'WAITING_FOR_CAMERA_INFO' if self.camera is None else
                  'SOURCE_STALE' if not ready else
                  'TARGET_CONFIRMED' if self.session.frames.confirmed else
                  'TARGET_CONFIGURED' if self.session.episode_id else 'WAIT_FOR_TARGET')
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetectionAdapter()
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
