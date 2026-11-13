"""Interactive, non-navigating camera/LiDAR alignment workflow."""

import argparse
from collections import deque
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from ee3033_search_policy_interfaces.msg import TargetObservation
from ee3033_search_policy_interfaces.srv import SetTargetClass

from .camera_lidar_calibration_core import (
    CalibrationSample,
    summarize_samples,
)
from .real_robot_workflow import BLOCKING_NODES, ManagedProcess, WorkflowError
from .stamped_localization import (
    bbox_requires_lidar,
    map_ground_point,
    rotation_from_quaternion,
)
from .target_localization_core import (
    nearest_supported_lidar_cluster,
    undistort_normalized,
)
from .yolo_bearing import RawCameraCalibration


CALIBRATION_BLOCKING_NODES = BLOCKING_NODES | frozenset({
    '/continuous_observation_adapter',
    '/ee3033_camera_lidar_calibration',
    '/euler_camera_tf',
})


class CalibrationNode(Node):
    def __init__(self, calibration_file, sample_count=5):
        super().__init__('ee3033_camera_lidar_calibration')
        self.calibration = RawCameraCalibration.load(calibration_file)
        self.sample_count = int(sample_count)
        self.camera = None
        self.scans = deque(maxlen=50)
        self.samples = deque(maxlen=100)
        self.last_observation_stamp = None
        self.last_error = 'waiting for target observations'
        self.receipts = {}
        self.adapter_status = ''
        self.lock = threading.Lock()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, '/mission/state', latched_qos)
        self.target_client = self.create_client(SetTargetClass, '/target/set_class')
        self.create_subscription(CameraInfo, '/camera_info', self._camera, sensor_qos)
        self.create_subscription(Image, '/camera/image_raw',
                                 lambda msg: self._touch('image'), sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._scan, sensor_qos)
        self.create_subscription(String, '/yolo/detections',
                                 lambda msg: self._touch('detection'), sensor_qos)
        self.create_subscription(Vector3Stamped, '/robot_euler',
                                 lambda msg: self._touch('euler'), sensor_qos)
        self.create_subscription(String, '/target/adapter_status',
                                 self._adapter_status, latched_qos)
        self.create_subscription(TargetObservation, '/target/observation',
                                 self._observation, sensor_qos)

    def _touch(self, name):
        self.receipts[name] = time.monotonic()

    def recent(self, name, age=2.0):
        return name in self.receipts and time.monotonic()-self.receipts[name] <= age

    def _adapter_status(self, msg):
        self.adapter_status = msg.data.strip().upper()
        self._touch('adapter')

    def _camera(self, msg):
        try:
            self.calibration.validate_camera_info(msg)
            self.camera = msg
            self._touch('camera_info')
        except ValueError as exc:
            self.camera = None
            self.last_error = 'camera calibration mismatch: '+str(exc)

    def _scan(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (math.isfinite(stamp) and stamp > 0 and msg.header.frame_id and
                msg.ranges and math.isfinite(msg.angle_min) and
                math.isfinite(msg.angle_increment) and msg.angle_increment != 0):
            self.scans.append((stamp, msg))
            self._touch('scan')

    def publish_state(self, state):
        self.state_pub.publish(String(data=str(state)))

    @staticmethod
    def _wait_future(future, timeout, description):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise WorkflowError(f'Timed out waiting for {description}')

    def configure_target(self, class_name, confidence, timeout=15.0):
        if not self.target_client.wait_for_service(timeout_sec=timeout):
            raise WorkflowError('/target/set_class service unavailable')
        request = SetTargetClass.Request()
        request.class_name = class_name
        request.min_confidence = float(confidence)
        future = self.target_client.call_async(request)
        self._wait_future(future, timeout, '/target/set_class')
        response = future.result()
        if response is None or not response.success:
            reason = response.message if response is not None else 'no response'
            raise WorkflowError('Target configuration rejected: '+reason)
        return response

    def _matched_scan(self, image_stamp, now):
        candidates = [
            (abs(stamp-image_stamp), scan)
            for stamp, scan in self.scans
            if 0 <= now-stamp <= .75
        ]
        if not candidates:
            return None
        delta, scan = min(candidates, key=lambda row: row[0])
        return scan if delta <= .12 else None

    def _ray(self, rotation, x, y):
        angle = self.calibration.bearing(x, y)
        camera = self.calibration.camera
        xy = undistort_normalized(
            (x-camera.cx)/camera.fx,
            (y-camera.cy)/camera.fy,
            camera.distortion,
            iterations=100,
        )
        ray = np.array([math.tan(angle), xy[1], 1.0])
        return rotation @ (ray/np.linalg.norm(ray))

    def _sample(self, msg):
        if not msg.confirmed:
            raise ValueError('TARGET_NOT_CONFIRMED')
        if self.camera is None:
            raise ValueError('WAITING_FOR_CAMERA_INFO')
        if (msg.header.frame_id != self.camera.header.frame_id or
                (msg.image_width, msg.image_height) !=
                (self.camera.width, self.camera.height)):
            raise ValueError('CAMERA_FRAME_OR_SIZE_MISMATCH')
        if bbox_requires_lidar(
                msg.center_x, msg.center_y, msg.width, msg.height,
                msg.image_width, msg.image_height):
            raise ValueError('BBOX_TOUCHES_TOP_OR_BOTTOM; move target farther away')
        now = self.get_clock().now().nanoseconds*1e-9
        image_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not 0 <= now-image_stamp <= .5:
            raise ValueError('STALE_TARGET_OBSERVATION')
        camera_tf = self.buffer.lookup_transform(
            'base_footprint', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        q = camera_tf.transform.rotation
        t = camera_tf.transform.translation
        camera_rotation = rotation_from_quaternion(q.x, q.y, q.z, q.w)
        camera_origin = np.array([t.x, t.y, t.z], dtype=float)
        centre = self._ray(camera_rotation, msg.center_x, msg.center_y)
        bottom = self._ray(
            camera_rotation, msg.center_x, msg.center_y+msg.height/2)
        edges = (
            self._ray(camera_rotation, msg.center_x-msg.width/2, msg.center_y),
            self._ray(camera_rotation, msg.center_x+msg.width/2, msg.center_y),
        )
        ground = map_ground_point(camera_origin, bottom, 0.0, .15, 4.0)
        if ground is None:
            raise ValueError('CAMERA_GROUND_PROJECTION_UNAVAILABLE')
        scan = self._matched_scan(image_stamp, now)
        if scan is None:
            raise ValueError('NO_SYNCHRONIZED_LIDAR_SCAN')
        scan_tf = self.buffer.lookup_transform(
            'base_footprint', scan.header.frame_id, Time.from_msg(scan.header.stamp))
        sq = scan_tf.transform.rotation
        st = scan_tf.transform.translation
        scan_rotation = rotation_from_quaternion(sq.x, sq.y, sq.z, sq.w)

        def scan_bearing(direction):
            transformed = scan_rotation.T @ np.asarray(direction, dtype=float)
            if np.linalg.norm(transformed[:2]) < 1e-9:
                raise ValueError('INVALID_SCAN_DIRECTION')
            return math.atan2(transformed[1], transformed[0])

        centre_scan_bearing = scan_bearing(centre)
        edge_bearings = tuple(scan_bearing(direction) for direction in edges)
        angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
        minimum_range = max(.15, float(scan.range_min))
        maximum_range = min(4.0, float(scan.range_max))
        selected = nearest_supported_lidar_cluster(
            scan.ranges, angles, centre_scan_bearing, edge_bearings,
            .05236, minimum_range, maximum_range, minimum_points=2)
        if selected is None:
            raise ValueError('NO_SUPPORTED_LIDAR_CLUSTER_IN_BBOX_SECTOR')
        raw_lidar_range, selected_angle = selected
        scan_point = np.array([
            raw_lidar_range*math.cos(selected_angle),
            raw_lidar_range*math.sin(selected_angle),
            0.0,
        ])
        lidar_origin = np.array([st.x, st.y, st.z], dtype=float)
        lidar_point = lidar_origin+scan_rotation@scan_point
        camera_range = float(np.linalg.norm(ground))
        lidar_range = float(np.linalg.norm(lidar_point[:2]))
        horizontal_from_camera = float(np.linalg.norm(
            ground-camera_origin[:2]))
        horizontal_to_lidar = float(np.linalg.norm(
            lidar_point[:2]-camera_origin[:2]))
        camera_depression = math.atan2(
            -float(bottom[2]), float(np.linalg.norm(bottom[:2])))
        required_depression = math.atan2(
            float(camera_origin[2]), horizontal_to_lidar)
        if min(horizontal_from_camera, horizontal_to_lidar) <= 0:
            raise ValueError('INVALID_PLANAR_RANGE')
        return CalibrationSample(
            camera_bearing_rad=math.atan2(float(centre[1]), float(centre[0])),
            lidar_bearing_rad=math.atan2(
                float(lidar_point[1]), float(lidar_point[0])),
            camera_range_m=camera_range,
            lidar_range_m=lidar_range,
            camera_depression_rad=camera_depression,
            required_depression_rad=required_depression,
            confidence=float(msg.confidence),
        )

    def _observation(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.last_observation_stamp is not None and stamp <= self.last_observation_stamp:
            return
        self.last_observation_stamp = stamp
        try:
            sample = self._sample(msg)
        except (ValueError, TypeError, TransformException) as exc:
            with self.lock:
                self.last_error = str(exc)
            return
        with self.lock:
            self.samples.append(sample)
            self.last_error = ''

    def take_measurement(self, timeout=20.0):
        with self.lock:
            self.samples.clear()
            self.last_error = 'waiting for synchronized camera/LiDAR samples'
        deadline = time.monotonic()+timeout
        next_report = time.monotonic()+2.0
        while time.monotonic() < deadline:
            with self.lock:
                samples = list(self.samples)
                last_error = self.last_error
            if len(samples) >= self.sample_count:
                return summarize_samples(samples[-self.sample_count:])
            if time.monotonic() >= next_report:
                print(
                    f'[CAL] collecting {len(samples)}/{self.sample_count}: '
                    f'{last_error or "ready"}', flush=True)
                next_report = time.monotonic()+2.0
            time.sleep(.1)
        raise WorkflowError(
            f'Measurement timed out ({len(samples)}/{self.sample_count}): '
            f'{last_error}')


class CalibrationWorkflow:
    def __init__(self, args):
        self.args = args
        self.environment = os.environ.copy()
        timestamp = time.strftime('%Y%m%dT%H%M%S')
        self.log_dir = Path(tempfile.gettempdir()) / f'ee3033_calibration_{timestamp}'
        self.log_dir.mkdir(parents=True, exist_ok=False)
        self.processes = []
        self.node = CalibrationNode(Path(args.calibration_file), args.samples)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def info(self, message):
        print(f'[CAL] {message}', flush=True)

    def start_process(self, name, command):
        process = ManagedProcess(name, command, self.log_dir, self.environment)
        self.processes.append(process)
        self.info(f'{name} started (pid={process.process.pid}, log={process.log_path})')
        return process

    @staticmethod
    def assert_alive(processes):
        for process in processes:
            code = process.poll()
            if code is not None:
                raise WorkflowError(
                    f'{process.name} exited with code {code}\n{process.tail()}')

    def wait_for(self, description, predicate, timeout, processes):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.assert_alive(processes)
            if predicate():
                self.info(description+': ready')
                return
            time.sleep(.2)
        details = '\n'.join(
            f'--- {process.name} ---\n{process.tail(12)}' for process in processes)
        raise WorkflowError(f'Timed out waiting for {description}\n{details}')

    @staticmethod
    def _direction(value, positive, negative, tolerance=math.radians(.1)):
        if abs(value) < tolerance:
            return '无需调整'
        return positive if value > 0 else negative

    def print_measurement(self, summary):
        degrees = math.degrees
        horizontal_direction = self._direction(
            summary.horizontal_adjustment_rad, '向左转相机', '向右转相机')
        vertical_direction = self._direction(
            summary.vertical_adjustment_rad, '向下压相机', '向上抬相机')
        camera_side = self._direction(
            summary.horizontal_adjustment_rad,
            '相机结果在雷达左侧', '相机结果在雷达右侧')
        range_side = ('相机测得更远' if summary.range_error_m > 0 else
                      '相机测得更近' if summary.range_error_m < 0 else '距离一致')
        print('', flush=True)
        self.info(f'测量完成：{summary.sample_count} 帧，中位置信度 '
                  f'{summary.confidence:.3f}')
        self.info(
            f'相机：方位 {degrees(summary.camera_bearing_rad):+.2f} deg，'
            f'地面距离 {summary.camera_range_m:.3f} m')
        self.info(
            f'雷达：方位 {degrees(summary.lidar_bearing_rad):+.2f} deg，'
            f'目标簇距离 {summary.lidar_range_m:.3f} m')
        self.info(
            f'左右偏差：{degrees(summary.horizontal_adjustment_rad):+.2f} deg '
            f'({camera_side})')
        self.info(
            f'距离偏差：{summary.range_error_m:+.3f} m ({range_side})')
        self.info(
            f'建议机械调整：{horizontal_direction} '
            f'{abs(degrees(summary.horizontal_adjustment_rad)):.2f} deg；'
            f'{vertical_direction} '
            f'{abs(degrees(summary.vertical_adjustment_rad)):.2f} deg（上下为近似值）')
        print('', flush=True)

    def run(self):
        self.info(f'logs: {self.log_dir}')
        self.info('navigation disabled: this tool never starts Nav2 or search nodes')
        base = self.start_process('base_lidar', [
            'ros2', 'launch', 'balance_bringup',
            'balance_slam_bringup.launch.py',
        ])
        self.wait_for(
            'base, LiDAR and balancing attitude',
            lambda: self.node.recent('scan', 1.0) and self.node.recent('euler', 1.0),
            self.args.startup_timeout, (base,))
        camera = self.start_process('camera', [
            'ros2', 'run', 'usb_cam', 'usb_cam_node_exe', '--ros-args',
            '-r', '__node:=usb_cam',
            '-p', 'video_device:=/dev/video0',
            '-p', 'framerate:=30.0',
            '-p', 'image_width:=640', '-p', 'image_height:=480',
            '-p', 'pixel_format:=mjpeg2rgb',
            '-p', 'camera_name:=ee3033_camera',
            '-p', f'camera_info_url:=file://{self.args.camera_info}',
            '-r', '/image_raw:=/camera/image_raw',
        ])
        self.wait_for(
            'camera image and calibrated CameraInfo',
            lambda: self.node.recent('image') and self.node.recent('camera_info'),
            self.args.startup_timeout, (base, camera))
        yolo = self.start_process('yolo', [sys.executable, self.args.yolo_script])
        self.wait_for(
            'YOLO source', lambda: self.node.recent('detection'),
            self.args.startup_timeout, (base, camera, yolo))
        support = self.start_process('calibration_support', [
            'ros2', 'launch', 'ee3033_search_policy',
            'camera_lidar_calibration_support.launch.py',
            f'params_file:={self.args.params_file}',
            f'labels_file:={self.args.labels_file}',
            f'calibration_file:={self.args.calibration_file}',
        ])
        processes = (base, camera, yolo, support)
        deadline = time.monotonic()+self.args.startup_timeout
        while time.monotonic() < deadline:
            self.assert_alive(processes)
            self.node.publish_state('WAIT_FOR_TARGET')
            if self.node.adapter_status == 'WAIT_FOR_TARGET':
                break
            time.sleep(.2)
        else:
            raise WorkflowError(
                'Detector adapter did not reach WAIT_FOR_TARGET; status='+
                repr(self.node.adapter_status))
        response = self.node.configure_target(
            self.args.target_class, self.args.min_confidence)
        self.info(
            f'target configured: {response.normalized_class_name} '
            f'(class_id={response.class_id})')
        self.node.publish_state('SEARCHING')
        self.info('keep robot and target fixed; taking first measurement now')
        self.print_measurement(
            self.node.take_measurement(self.args.measurement_timeout))
        while True:
            input('[OPERATOR] 调整相机后按 ENTER 再测一次；Ctrl+C 退出：')
            self.assert_alive(processes)
            self.print_measurement(
                self.node.take_measurement(self.args.measurement_timeout))

    def shutdown(self):
        for process in reversed(self.processes):
            try:
                process.stop()
            except Exception as exc:
                self.info(f'cleanup warning for {process.name}: {exc}')
        self.processes.clear()
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_args(argv=None):
    share = Path(get_package_share_directory('ee3033_search_policy'))
    parser = argparse.ArgumentParser(
        description='Iterative camera/LiDAR physical alignment tool')
    parser.add_argument('--target-class', required=True)
    parser.add_argument('--min-confidence', type=float, default=.55)
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--startup-timeout', type=float, default=40.)
    parser.add_argument('--measurement-timeout', type=float, default=20.)
    parser.add_argument('--params-file', default=str(
        share/'config'/'real_robot_workflow.yaml'))
    parser.add_argument('--labels-file', default=str(
        share/'config'/'yolo_ncnn_coco80_labels.json'))
    parser.add_argument('--calibration-file', default=str(
        share/'calibration'/'calibration.npz'))
    parser.add_argument('--camera-info', default=(
        '/home/ros2_ws/src/ee3033-search-policy/results/'
        'updated_calibration/camera_info.yaml'))
    parser.add_argument('--yolo-script', default='/home/ros2_ws/scripts/yolo_ncnn_node.py')
    return parser.parse_args(argv)


def preflight(args):
    if not sys.stdin.isatty():
        raise WorkflowError('Interactive terminal required')
    required = (
        args.params_file, args.labels_file, args.calibration_file,
        args.camera_info, args.yolo_script,
    )
    missing = [str(path) for path in required if not Path(path).is_file()]
    if missing:
        raise WorkflowError('Required file(s) missing: '+', '.join(missing))
    if not Path('/dev/video0').exists():
        raise WorkflowError('/dev/video0 is missing')
    if (not 0 < args.min_confidence <= 1 or args.samples < 1 or
            min(args.startup_timeout, args.measurement_timeout) <= 0):
        raise WorkflowError('Invalid confidence, sample count, or timeout')
    result = subprocess.run(
        ['ros2', 'node', 'list'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=8., check=False)
    nodes = set(result.stdout.splitlines())
    blocking = sorted(nodes & CALIBRATION_BLOCKING_NODES)
    if blocking:
        raise WorkflowError('Stop duplicate robot nodes first: '+', '.join(blocking))
    prefix = subprocess.run(
        ['ros2', 'pkg', 'prefix', 'usb_cam'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=8., check=False)
    expected = '/home/ros2_ws/usb_cam_timestamp_test.hZhIXi/install/usb_cam'
    if prefix.returncode != 0 or prefix.stdout.strip() != expected:
        raise WorkflowError(
            f'Timestamp-correct usb_cam overlay is not active; expected {expected}, '
            f'got {prefix.stdout.strip()!r}')


def main(argv=None):
    args = parse_args(argv)
    workflow = None
    try:
        preflight(args)
        rclpy.init()
        workflow = CalibrationWorkflow(args)
        workflow.run()
    except KeyboardInterrupt:
        print('\n[CAL] operator exit requested', flush=True)
    except (WorkflowError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f'\n[CAL] STOP: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if workflow is not None:
            workflow.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
