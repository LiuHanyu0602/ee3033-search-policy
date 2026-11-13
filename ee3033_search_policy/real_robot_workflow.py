"""Interactive one-command new- or saved-map search and return workflow."""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from ee3033_search_policy_interfaces.msg import TargetEstimate
from ee3033_search_policy_interfaces.srv import SetTargetClass

from .workflow_support import (
    PlanarPose,
    TERMINAL_MISSION_STATES,
    angular_error,
    clean_isolated_occupied_pixels,
    file_sha256,
    load_saved_home_pose,
    saved_map_files,
    validate_existing_map_yaml,
    validate_saved_map_thresholds,
    planar_pose_error,
    validate_new_map_prefix,
)


BLOCKING_NODES = frozenset({
    '/slam_toolbox', '/map_server', '/amcl', '/planner_server',
    '/controller_server', '/bt_navigator', '/balance_robot_node',
    '/scan_resampler', '/usb_cam', '/yolo_ncnn',
    '/target_mission_coordinator', '/search_manager',
})


class WorkflowError(RuntimeError):
    """A fail-closed workflow error with an operator-facing message."""


class ManagedProcess:
    def __init__(self, name, command, log_dir, environment):
        self.name = name
        self.command = list(command)
        self.log_path = Path(log_dir) / f'{name}.log'
        self.stream = self.log_path.open('w', encoding='utf-8')
        self.process = subprocess.Popen(
            self.command,
            stdout=self.stream,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
            text=True,
        )

    def poll(self):
        return self.process.poll()

    def tail(self, lines=24):
        self.stream.flush()
        try:
            return '\n'.join(self.log_path.read_text(
                encoding='utf-8', errors='replace').splitlines()[-lines:])
        except OSError:
            return '<log unavailable>'

    def stop(self, timeout=15.0):
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.process.wait(timeout=timeout)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if self.process.poll() is None:
                    try:
                        os.killpg(self.process.pid, signal.SIGTERM)
                        self.process.wait(timeout=5.0)
                    except (ProcessLookupError, subprocess.TimeoutExpired):
                        if self.process.poll() is None:
                            os.killpg(self.process.pid, signal.SIGKILL)
                            self.process.wait(timeout=3.0)
        self.stream.close()

    def detach(self):
        """Leave the child running but release this process's log handle."""
        if not self.stream.closed:
            self.stream.close()


class WorkflowNode(Node):
    def __init__(self):
        super().__init__('ee3033_real_robot_workflow')
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
        realtime_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.receipts = {}
        self.detection_receipts = deque(maxlen=60)
        self.mission_state = ''
        self.adapter_status = ''
        self.localization_status = ''
        self.localization_candidate = None
        self.runtime = {}
        self.estimate = None
        self.amcl_pose = None
        self.amcl_pose_serial = 0
        self.amcl_pose_history = deque(maxlen=20)
        # Lifecycle readiness is polled while a separate executor thread is
        # spinning this node.  Keep these clients for the node lifetime: in
        # Humble, destroying and recreating them inside the polling loop can
        # race the executor wait set and raise InvalidHandle.
        self.lifecycle_clients = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', QoSProfile(depth=5))
        self.create_subscription(
            OccupancyGrid, '/map', lambda msg: self._touch('map'), latched_qos)
        self.create_subscription(
            LaserScan, '/scan', lambda msg: self._touch('scan'), sensor_qos)
        self.create_subscription(
            Odometry, '/odom', lambda msg: self._touch('odom'), sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose,
            QoSProfile(depth=10))
        self.create_subscription(
            Image, '/camera/image_raw', lambda msg: self._touch('image'), sensor_qos)
        self.create_subscription(
            CameraInfo, '/camera_info', lambda msg: self._touch('camera_info'), sensor_qos)
        self.create_subscription(
            String, '/yolo/detections', self._detection, sensor_qos)
        self.create_subscription(
            String, '/mission/state', self._mission, latched_qos)
        self.create_subscription(
            String, '/target/adapter_status', self._adapter, latched_qos)
        self.create_subscription(
            String, '/target/localization_status', self._localization_status,
            latched_qos)
        self.create_subscription(
            TargetEstimate, '/target/estimate_candidate',
            self._localization_candidate, realtime_qos)
        self.create_subscription(
            String, '/search/runtime_state', self._runtime_callback, realtime_qos)
        self.create_subscription(
            TargetEstimate, '/target/estimate', self._estimate_callback, latched_qos)
        self.return_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.active_return_handle = None

    def _touch(self, name):
        self.receipts[name] = time.monotonic()

    def _detection(self, msg):
        del msg
        now = time.monotonic()
        self.receipts['detection'] = now
        self.detection_receipts.append(now)

    def _mission(self, msg):
        self.mission_state = msg.data.strip().upper()
        self._touch('mission')

    def _adapter(self, msg):
        self.adapter_status = msg.data.strip().upper()
        self._touch('adapter')

    def _localization_status(self, msg):
        self.localization_status = msg.data.strip()
        self._touch('localization_status')

    def _localization_candidate(self, msg):
        self.localization_candidate = msg
        self._touch('localization_candidate')

    def _runtime_callback(self, msg):
        try:
            self.runtime = json.loads(msg.data)
            self._touch('runtime')
        except (ValueError, TypeError):
            pass

    def _estimate_callback(self, msg):
        self.estimate = msg
        self._touch('estimate')

    def _amcl_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if (msg.header.frame_id != 'map' or
                not all(math.isfinite(value) for value in
                        (p.x, p.y, q.x, q.y, q.z, q.w)) or
                abs(norm-1.0) > 1e-3):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.amcl_pose = PlanarPose(float(p.x), float(p.y), float(yaw))
        self.amcl_pose_serial += 1
        self.amcl_pose_history.append(
            (self.amcl_pose_serial, self.amcl_pose))
        self._touch('amcl_pose')

    def recent(self, name, age=2.0):
        return name in self.receipts and time.monotonic()-self.receipts[name] <= age

    def detection_rate(self):
        if len(self.detection_receipts) < 2:
            return 0.0
        duration = self.detection_receipts[-1]-self.detection_receipts[0]
        return ((len(self.detection_receipts)-1)/duration) if duration > 0 else 0.0

    def pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        q = transform.transform.rotation
        t = transform.transform.translation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not all(math.isfinite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w)):
            return None
        if abs(norm-1.0) > 1e-3:
            return None
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return PlanarPose(float(t.x), float(t.y), float(yaw))

    def publish_initial_pose(self, pose):
        self.amcl_pose = None
        self.amcl_pose_history.clear()
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = math.sin(pose.yaw/2.0)
        message.pose.pose.orientation.w = math.cos(pose.yaw/2.0)
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = math.radians(15.0)**2
        for _ in range(5):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(message)
            time.sleep(0.2)

    def call_trigger(self, service_name, timeout=10.0):
        client = self.create_client(Trigger, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise WorkflowError(f'Service unavailable: {service_name}')
            future = client.call_async(Trigger.Request())
            self._wait_future(future, timeout, service_name)
            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else 'no response'
                raise WorkflowError(f'{service_name} rejected: {message}')
            return response.message
        finally:
            self.destroy_client(client)

    def set_target(self, class_name, confidence, timeout=10.0):
        client = self.create_client(SetTargetClass, '/target/set_class')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise WorkflowError('Service unavailable: /target/set_class')
            request = SetTargetClass.Request()
            request.class_name = class_name
            request.min_confidence = float(confidence)
            future = client.call_async(request)
            self._wait_future(future, timeout, '/target/set_class')
            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else 'no response'
                raise WorkflowError(f'/target/set_class rejected: {message}')
            return response
        finally:
            self.destroy_client(client)

    @staticmethod
    def _wait_future(future, timeout, description):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise WorkflowError(f'Timed out waiting for {description}')

    def lifecycle_active(self, node_name, timeout=2.0):
        client = self.lifecycle_clients.get(node_name)
        if client is None:
            client = self.create_client(GetState, f'/{node_name}/get_state')
            self.lifecycle_clients[node_name] = client
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        future = client.call_async(GetState.Request())
        try:
            self._wait_future(future, timeout, node_name)
        except WorkflowError:
            return False
        response = future.result()
        return response is not None and response.current_state.label == 'active'

    def return_home(self, pose, timeout=180.0):
        if not self.return_client.wait_for_server(timeout_sec=10.0):
            raise WorkflowError('/navigate_to_pose action server unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        goal.pose.pose.orientation.z = math.sin(pose.yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(pose.yaw/2.0)
        send_future = self.return_client.send_goal_async(goal)
        self._wait_future(send_future, 10.0, 'return-home goal acceptance')
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise WorkflowError('Nav2 rejected the return-home goal')
        self.active_return_handle = handle
        result_future = handle.get_result_async()
        try:
            self._wait_future(result_future, timeout, 'return-home result')
        except WorkflowError:
            cancel_future = handle.cancel_goal_async()
            self._wait_future(cancel_future, 10.0, 'return-home cancellation')
            self._wait_future(result_future, 15.0, 'return-home canceled terminal')
            self.active_return_handle = None
            raise
        wrapped = result_future.result()
        self.active_return_handle = None
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = wrapped.status if wrapped is not None else 'missing'
            raise WorkflowError(f'Return-home finished with status {status}, not SUCCEEDED')


class RealRobotWorkflow:
    def __init__(self, args):
        self.args = args
        self.existing_map = args.existing_map is not None
        if self.existing_map:
            self.map_prefix = validate_existing_map_yaml(args.existing_map)
            self.home, self.home_source = load_saved_home_pose(
                self.map_prefix,
                args.initial_pose_x, args.initial_pose_y, args.initial_pose_yaw)
        else:
            self.map_prefix = validate_new_map_prefix(args.map_prefix)
            self.home = None
            self.home_source = 'mapping start'
        self.environment = os.environ.copy()
        timestamp = time.strftime('%Y%m%dT%H%M%S')
        self.log_dir = Path(tempfile.gettempdir()) / f'ee3033_workflow_{timestamp}'
        self.log_dir.mkdir(parents=True, exist_ok=False)
        if self.existing_map:
            self.summary_path = Path(
                str(self.map_prefix)+f'.search_{timestamp}.run.json')
        else:
            self.summary_path = Path(str(self.map_prefix)+'.run.json')
        self.summary = {
            'started_at': timestamp,
            'map_prefix': str(self.map_prefix),
            'map_mode': 'existing' if self.existing_map else 'new',
            'status': 'STARTING',
        }
        self.processes = []
        self.node = WorkflowNode()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def info(self, message):
        print(f'[EE3033] {message}', flush=True)

    def start_process(self, name, command):
        process = ManagedProcess(name, command, self.log_dir, self.environment)
        self.processes.append(process)
        self.info(f'{name} started (pid={process.process.pid}, log={process.log_path})')
        return process

    def assert_alive(self, processes):
        for process in processes:
            code = process.poll()
            if code is not None:
                raise WorkflowError(
                    f'{process.name} exited with code {code}\n{process.tail()}')

    def wait_for(self, description, predicate, timeout, processes=()):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.assert_alive(processes)
            if predicate():
                self.info(f'{description}: ready')
                return
            time.sleep(0.2)
        details = '\n'.join(
            f'--- {process.name} ---\n{process.tail(12)}' for process in processes)
        raise WorkflowError(f'Timed out waiting for {description}\n{details}')

    def capture_pose(self, description, processes=(), sample_count=7):
        samples = []
        deadline = time.monotonic()+3.0
        while len(samples) < sample_count and time.monotonic() < deadline:
            self.assert_alive(processes)
            pose = self.node.pose()
            if pose is not None:
                samples.append(pose)
            time.sleep(0.1)
        if len(samples) < sample_count:
            raise WorkflowError(f'Could not capture stable {description} pose')
        yaw = math.atan2(
            sum(math.sin(pose.yaw) for pose in samples),
            sum(math.cos(pose.yaw) for pose in samples),
        )
        return PlanarPose(
            statistics.median(pose.x for pose in samples),
            statistics.median(pose.y for pose in samples),
            yaw,
        )

    def wait_for_amcl_convergence(
            self, expected, after_serial, timeout, processes):
        deadline = time.monotonic()+timeout
        last_serial = after_serial
        consecutive = 0
        while time.monotonic() < deadline:
            self.assert_alive(processes)
            updates = [entry for entry in self.node.amcl_pose_history
                       if entry[0] > last_serial]
            for serial, amcl in updates:
                last_serial = serial
                tf_pose = self.node.pose()
                amcl_ok = amcl is not None and all(
                    measured <= limit for measured, limit in zip(
                        planar_pose_error(amcl, expected),
                        (self.args.start_position_tolerance_m,
                         self.args.start_yaw_tolerance_rad)))
                tf_ok = tf_pose is not None and all(
                    measured <= limit for measured, limit in zip(
                        planar_pose_error(tf_pose, expected),
                        (self.args.start_position_tolerance_m,
                         self.args.start_yaw_tolerance_rad)))
                consecutive = consecutive+1 if amcl_ok and tf_ok else 0
                if consecutive >= 3:
                    self.info('AMCL localization at mapping start: ready')
                    return
            time.sleep(0.1)
        raise WorkflowError(
            'AMCL did not converge for three consecutive updates near the '
            'recorded mapping start pose')

    def prompt_start(self):
        if not sys.stdin.isatty():
            raise WorkflowError('Interactive terminal required for operator confirmations')
        if self.existing_map:
            input(
                '[OPERATOR] Place the robot near the saved initial pose '
                f'(x={self.home.x:.3f}, y={self.home.y:.3f}, '
                f'yaw={self.home.yaw:.3f}), clear the floor, keep the E-stop '
                'ready, then press ENTER: ')
            return
        input(
            '[OPERATOR] Clear the floor, keep the E-stop ready, then press ENTER '
            'to start the real robot workflow: ')

    def prepare_map(self):
        if self.existing_map:
            yaml_path, pgm_path = saved_map_files(self.map_prefix)
            validate_saved_map_thresholds(yaml_path)
            self.summary['home'] = {
                'x': self.home.x, 'y': self.home.y, 'yaw': self.home.yaw}
            self.summary['existing_map_source'] = {
                'yaml': str(yaml_path),
                'pgm': str(pgm_path),
                'yaml_sha256': file_sha256(yaml_path),
                'pgm_sha256': file_sha256(pgm_path),
                'home_source': self.home_source,
            }
            self._write_summary()
            self.info(
                f'existing map selected: {yaml_path}; '
                f'home pose source: {self.home_source}')
            return self.home

        mapping = self.start_process('mapping', [
            'ros2', 'launch', 'balance_bringup',
            'balance_slam_bringup.launch.py',
        ])
        self.wait_for(
            'SLAM map/scan/odom/TF',
            lambda: (self.node.recent('map', 5.0) and
                     self.node.recent('scan', 1.0) and
                     self.node.recent('odom', 1.0) and
                     self.node.pose() is not None),
            self.args.startup_timeout,
            (mapping,),
        )
        mapping_start = self.capture_pose('mapping start', (mapping,))
        self.info(
            f'mapping start recorded: x={mapping_start.x:.3f}, '
            f'y={mapping_start.y:.3f}, yaw={mapping_start.yaw:.3f}')
        self.info('SLAM is live. Map manually with the existing operator controls.')
        while True:
            answer = input(
                '[OPERATOR] After loop closure, drive back near the mapping '
                'start, stop manual commands, then type DONE: '
            ).strip().casefold()
            if answer != 'done':
                self.info('Mapping continues; return to the recorded start before DONE.')
                continue
            mapping_return = self.capture_pose('mapping return', (mapping,))
            position_error, yaw_error = planar_pose_error(
                mapping_return, mapping_start)
            self.info(
                f'mapping return check: distance={position_error:.3f} m, '
                f'yaw_error={yaw_error:.3f} rad')
            if (position_error <= self.args.start_position_tolerance_m and
                    yaw_error <= self.args.start_yaw_tolerance_rad):
                break
            self.info(
                'Robot is not close enough to the mapping start; mapping remains '
                'active so the operator can correct the pose.')
        home = mapping_start
        self.summary['mapping_start'] = {
            'x': mapping_start.x, 'y': mapping_start.y, 'yaw': mapping_start.yaw}
        self.summary['mapping_return'] = {
            'x': mapping_return.x, 'y': mapping_return.y, 'yaw': mapping_return.yaw,
            'position_error_m': position_error, 'yaw_error_rad': yaw_error}
        self.summary['home'] = {'x': home.x, 'y': home.y, 'yaw': home.yaw}

        self._save_map()
        mapping.stop()
        self.processes.remove(mapping)
        self.info('mapping stack stopped; switching to saved-map Nav2')
        try:
            self.node.tf_buffer.clear()
        except AttributeError:
            pass
        time.sleep(2.0)
        return home

    def run(self):
        self.info(f'run logs: {self.log_dir}')
        if self.existing_map:
            self.info(f'existing map: {self.map_prefix}.yaml')
        else:
            self.info(f'new map: {self.map_prefix}.yaml/.pgm')
        self.prompt_start()
        home = self.prepare_map()

        nav = self.start_process('known_map_nav2', [
            'ros2', 'launch', 'balance_bringup',
            'balance_known_map_waypoint_bringup.launch.py',
            f'map:={self.map_prefix}.yaml', 'start_waypoints:=false',
        ])
        localization_nodes = ('map_server', 'amcl')
        self.wait_for(
            'saved map and localization lifecycle nodes',
            lambda: (self.node.recent('map', 5.0) and all(
                self.node.lifecycle_active(name) for name in localization_nodes)),
            self.args.nav_timeout,
            (nav,),
        )
        amcl_serial = self.node.amcl_pose_serial
        self.node.publish_initial_pose(home)
        self.wait_for_amcl_convergence(
            home, amcl_serial, self.args.nav_timeout, (nav,))
        navigation_nodes = (
            'planner_server', 'controller_server',
            'bt_navigator', 'velocity_smoother',
        )
        self.wait_for(
            'Nav2 navigation lifecycle nodes',
            lambda: all(
                self.node.lifecycle_active(name) for name in navigation_nodes),
            self.args.nav_timeout,
            (nav,),
        )
        if not self.node.return_client.wait_for_server(timeout_sec=10.0):
            raise WorkflowError('/navigate_to_pose action server unavailable')

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
            'camera image and CameraInfo',
            lambda: self.node.recent('image') and self.node.recent('camera_info'),
            self.args.startup_timeout,
            (nav, camera),
        )

        yolo = self.start_process('yolo', [sys.executable, self.args.yolo_script])
        self.wait_for(
            'YOLO detections',
            lambda: (self.node.recent('detection') and
                     len(self.node.detection_receipts) >= 5 and
                     3.5 <= self.node.detection_rate() <= 7.5),
            self.args.startup_timeout,
            (nav, camera, yolo),
        )
        self.info(f'YOLO rate: {self.node.detection_rate():.2f} Hz')

        runtime = self.start_process('search_runtime', [
            'ros2', 'launch', 'ee3033_search_policy',
            'yolo_complete_validation.launch.py',
            f'params_file:={self.args.params_file}',
            'navigation_enabled:=true',
            'single_goal_mode:=false', 'max_navigation_goals:=100',
            f'labels_file:={self.args.labels_file}',
            f'calibration_file:={self.args.calibration_file}',
        ])
        all_processes = (nav, camera, yolo, runtime)
        self.wait_for(
            'search runtime',
            lambda: (self.node.mission_state == 'IDLE' and
                     self.node.adapter_status == 'WAIT_FOR_TARGET' and
                     self._runtime_ready()),
            self.args.runtime_timeout,
            all_processes,
        )

        self.node.call_trigger('/mission/start_mapping')
        self.node.call_trigger('/mission/finish_mapping')
        response = None
        target = ''
        while response is None:
            target = input(
                '[OPERATOR] Place the target on the floor outside the initial '
                'camera view, then enter its class (example: bottle): '
            ).strip()
            if not target:
                continue
            try:
                response = self._configure_target(target)
            except WorkflowError as exc:
                if 'Unknown model category' not in str(exc):
                    raise
                self.info(str(exc))
                target = ''
        self.info(
            f'target configured: {response.normalized_class_name} '
            f'(class_id={response.class_id}, episode={response.episode_id})')
        self.summary['target'] = {
            'class_name': response.normalized_class_name,
            'class_id': response.class_id,
            'episode_id': response.episode_id,
            'min_confidence': self.args.min_confidence,
        }
        self.wait_for(
            'fresh configured detector',
            lambda: self.node.adapter_status == 'TARGET_CONFIGURED',
            15.0,
            all_processes,
        )
        self._start_search_when_ready()
        self.info('automatic MLP-guided search started')

        terminal = self._wait_for_search_terminal(all_processes)
        if terminal != 'FINISHED':
            raise WorkflowError(f'Search ended in {terminal}; automatic return not authorized')
        self.wait_for(
            'verified search terminal',
            lambda: (self.node.runtime.get('quiescent') is True and
                     self.node.runtime.get('unknown_terminal') is False),
            20.0,
            all_processes,
        )
        if self.node.estimate is None or not self.node.estimate.valid:
            raise WorkflowError('FINISHED without a retained valid target estimate')
        estimate = self.node.estimate
        start_to_target_distance = math.hypot(
            estimate.position.x-home.x,
            estimate.position.y-home.y,
        )
        self.info(
            f'target coordinate: x={estimate.position.x:.3f}, '
            f'y={estimate.position.y:.3f}, '
            f'method={estimate.method}')
        self.info(
            f'start coordinate: x={home.x:.3f}, y={home.y:.3f}')
        self.info(
            f'start-to-target Euclidean distance: '
            f'{start_to_target_distance:.3f} m')
        self.summary['estimate'] = {
            'x': estimate.position.x,
            'y': estimate.position.y,
            'z': estimate.position.z,
            'method': estimate.method,
            'quality': estimate.quality,
            'reason': estimate.reason,
        }
        self.info('returning to the mapping start pose')
        self.node.return_home(home, timeout=self.args.return_timeout)
        self.summary['status'] = 'COMPLETE'
        self.summary['return_home'] = 'SUCCEEDED'
        self._write_summary()
        self.info('RETURN_HOME_SUCCEEDED; robot reported SUCCEEDED and stopped')
        self.info('workflow complete; keep this command running while balance is required')
        self.info('press Ctrl+C only when it is safe to shut down the owned processes')
        while rclpy.ok():
            self.assert_alive(all_processes)
            time.sleep(1.0)

    def _configure_target(self, target, timeout=30.0):
        deadline = time.monotonic()+timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                return self.node.set_target(target, self.args.min_confidence)
            except WorkflowError as exc:
                last_error = exc
                if 'ready detector' not in str(exc):
                    raise
                time.sleep(0.25)
        raise WorkflowError(
            f'Target configuration did not obtain a fresh detector: {last_error}')

    def _start_search_when_ready(self, timeout=20.0):
        deadline = time.monotonic()+timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                return self.node.call_trigger('/mission/start_search')
            except WorkflowError as exc:
                last_error = exc
                if 'Require configured fresh detector' not in str(exc):
                    raise
                time.sleep(0.25)
        raise WorkflowError(f'Search readiness did not stabilize: {last_error}')

    def _save_map(self):
        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', str(self.map_prefix), '-t', '/map', '--fmt', 'pgm',
            '--ros-args', '-p', 'save_map_timeout:=10.0',
            '-p', 'free_thresh_default:=0.196',
            '-p', 'occupied_thresh_default:=0.65',
        ]
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=30.0,
                env=self.environment,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise WorkflowError('Map saver timed out') from exc
        if result.returncode != 0:
            raise WorkflowError('Map saver failed:\n'+result.stdout[-3000:])
        yaml_path, pgm_path = saved_map_files(self.map_prefix)
        validate_saved_map_thresholds(yaml_path)
        cleanup = clean_isolated_occupied_pixels(yaml_path, pgm_path)
        # Revalidate the rewritten map before any map server can consume it.
        saved_map_files(self.map_prefix)
        self.summary['map'] = {
            'yaml': str(yaml_path),
            'pgm': str(pgm_path),
            'yaml_sha256': file_sha256(yaml_path),
            'pgm_sha256': cleanup.cleaned_sha256,
            'raw_pgm': str(cleanup.raw_path),
            'raw_pgm_sha256': cleanup.raw_sha256,
            'isolated_occupied_cells_removed': cleanup.removed,
        }
        self.summary['status'] = 'MAP_SAVED'
        self._write_summary()
        self.info(
            f"map saved: yaml_sha256={self.summary['map']['yaml_sha256']}, "
            f"pgm_sha256={self.summary['map']['pgm_sha256']}, "
            f"isolated_cells_removed={cleanup.removed}")

    def _write_summary(self):
        temporary = Path(str(self.summary_path)+'.tmp')
        temporary.write_text(
            json.dumps(self.summary, indent=2, sort_keys=True)+'\n',
            encoding='utf-8')
        temporary.replace(self.summary_path)

    def record_failure(self, message):
        self.summary['status'] = 'FAILED'
        self.summary['failure'] = str(message)
        try:
            self._write_summary()
        except OSError:
            pass

    def _localized_near(self, expected):
        actual = self.node.pose()
        if actual is None:
            return False
        distance = math.hypot(actual.x-expected.x, actual.y-expected.y)
        return distance <= 0.50 and angular_error(actual.yaw, expected.yaw) <= 0.70

    def _runtime_ready(self):
        runtime = self.node.runtime
        return (runtime.get('ready') is True and
                runtime.get('quiescent') is True and
                runtime.get('unknown_terminal') is False and
                runtime.get('navigation_enabled') is True and
                runtime.get('operator_stop') is False and
                runtime.get('finished') is False)

    def _wait_for_search_terminal(self, processes):
        deadline = time.monotonic()+self.args.search_timeout
        last_state = None
        last_localization = None
        last_candidate_stamp = None
        while time.monotonic() < deadline:
            self.assert_alive(processes)
            state = self.node.mission_state
            if state and state != last_state:
                self.info(f'mission state: {state}')
                last_state = state
            localization = self.node.localization_status
            if (state in ('ALIGNING_TARGET', 'LOCALIZING_TARGET') and
                    localization and localization != last_localization):
                self.info(f'localization status: {localization}')
                last_localization = localization
            candidate = self.node.localization_candidate
            if candidate is not None and candidate.valid:
                candidate_stamp = (
                    candidate.header.stamp.sec, candidate.header.stamp.nanosec)
                if candidate_stamp != last_candidate_stamp:
                    self.info(
                        f'localization candidate: method={candidate.method}, '
                        f'x={candidate.position.x:.3f}, '
                        f'y={candidate.position.y:.3f}, '
                        f'reason={candidate.reason}')
                    last_candidate_stamp = candidate_stamp
            if state in TERMINAL_MISSION_STATES:
                return state
            time.sleep(0.2)
        raise WorkflowError('Timed out waiting for the search mission terminal state')

    def shutdown(self):
        if not self._verify_stopped_before_shutdown():
            self.info('owned processes left running because stop was not verified')
            self.info(f'inspect logs under {self.log_dir} and use the physical E-stop')
            for process in self.processes:
                process.detach()
        else:
            for process in reversed(self.processes):
                try:
                    process.stop()
                except Exception as exc:  # cleanup must continue for other children
                    self.info(f'cleanup warning for {process.name}: {exc}')
        self.processes.clear()
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _verify_stopped_before_shutdown(self):
        if self.node.mission_state == 'ALIGNING_TARGET':
            self.info('canceling active target-alignment Spin before shutdown')
            try:
                self.node.call_trigger('/target/cancel_alignment', timeout=10.0)
            except WorkflowError as exc:
                self.info(f'alignment cancel request failed: {exc}')
                return False
            deadline = time.monotonic()+15.0
            while time.monotonic() < deadline:
                if self.node.mission_state == 'OPERATOR_STOPPED':
                    break
                if self.node.mission_state == 'STOP_UNVERIFIED':
                    return False
                time.sleep(0.1)
            else:
                return False
        handle = self.node.active_return_handle
        if handle is not None:
            self.info('canceling active return-home goal before shutdown')
            try:
                future = handle.cancel_goal_async()
                self.node._wait_future(future, 10.0, 'return-home cancellation')
                result = handle.get_result_async()
                self.node._wait_future(result, 15.0, 'return-home terminal')
                wrapped = result.result()
                if wrapped is None or wrapped.status not in (
                        GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                    return False
                self.node.active_return_handle = None
            except WorkflowError as exc:
                self.info(f'return-home stop verification failed: {exc}')
                return False
        runtime = self.node.runtime
        if not runtime or (runtime.get('quiescent') is True and
                           runtime.get('unknown_terminal') is False):
            return True
        self.info('requesting search navigation cancellation before shutdown')
        try:
            self.node.call_trigger('/search/cancel_navigation', timeout=10.0)
        except WorkflowError as exc:
            self.info(f'cancel request failed: {exc}')
            return False
        deadline = time.monotonic()+15.0
        while time.monotonic() < deadline:
            runtime = self.node.runtime
            if (runtime.get('quiescent') is True and
                    runtime.get('unknown_terminal') is False):
                return True
            time.sleep(0.1)
        return False


def parse_args(argv=None):
    share = Path(get_package_share_directory('ee3033_search_policy'))
    parser = argparse.ArgumentParser(
        description='Interactive real SLAM, target search, and return-home workflow')
    map_group = parser.add_mutually_exclusive_group(required=True)
    map_group.add_argument(
        '--map-prefix', help='Absolute new map prefix, without .yaml/.pgm')
    map_group.add_argument(
        '--existing-map', help='Absolute existing map YAML; skips SLAM/map saving')
    parser.add_argument('--initial-pose-x', type=float)
    parser.add_argument('--initial-pose-y', type=float)
    parser.add_argument('--initial-pose-yaw', type=float,
                        help='Existing-map initial yaw in radians')
    parser.add_argument('--navigation-enabled', required=True, choices=('true',),
                        help='Explicit real-motion authorization; must be true')
    parser.add_argument('--min-confidence', type=float, default=0.55)
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
    parser.add_argument('--startup-timeout', type=float, default=30.0)
    parser.add_argument('--nav-timeout', type=float, default=45.0)
    parser.add_argument('--runtime-timeout', type=float, default=30.0)
    parser.add_argument('--search-timeout', type=float, default=330.0)
    parser.add_argument('--return-timeout', type=float, default=180.0)
    parser.add_argument('--start-position-tolerance-m', type=float, default=0.40)
    parser.add_argument('--start-yaw-tolerance-rad', type=float, default=0.52)
    return parser.parse_args(argv)


def preflight(args):
    required_files = (
        args.params_file, args.labels_file, args.calibration_file,
        args.camera_info, args.yolo_script,
    )
    missing = [str(path) for path in required_files if not Path(path).is_file()]
    if missing:
        raise WorkflowError('Required file(s) missing: '+', '.join(missing))
    if not Path('/dev/video0').exists():
        raise WorkflowError('/dev/video0 is missing')
    if not 0.0 < args.min_confidence <= 1.0:
        raise WorkflowError('--min-confidence must be in (0, 1]')
    pose_override = (
        args.initial_pose_x, args.initial_pose_y, args.initial_pose_yaw)
    if args.existing_map is None and any(value is not None for value in pose_override):
        raise WorkflowError(
            '--initial-pose-x/y/yaw are valid only with --existing-map')
    if args.existing_map is not None:
        prefix = validate_existing_map_yaml(args.existing_map)
        load_saved_home_pose(prefix, *pose_override)
    if (not math.isfinite(args.start_position_tolerance_m) or
            args.start_position_tolerance_m <= 0):
        raise WorkflowError('--start-position-tolerance-m must be positive')
    if (not math.isfinite(args.start_yaw_tolerance_rad) or
            not 0 < args.start_yaw_tolerance_rad <= math.pi):
        raise WorkflowError('--start-yaw-tolerance-rad must be in (0, pi]')
    result = subprocess.run(
        ['ros2', 'node', 'list'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=8.0, check=False)
    nodes = set(result.stdout.splitlines())
    blocking = sorted(nodes & BLOCKING_NODES)
    if blocking:
        raise WorkflowError('Stop duplicate robot nodes first: '+', '.join(blocking))
    prefix = subprocess.run(
        ['ros2', 'pkg', 'prefix', 'usb_cam'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=8.0, check=False)
    expected = '/home/ros2_ws/usb_cam_timestamp_test.hZhIXi/install/usb_cam'
    if prefix.returncode != 0 or prefix.stdout.strip() != expected:
        raise WorkflowError(
            f'Timestamp-correct usb_cam overlay is not active; expected {expected}, '
            f'got {prefix.stdout.strip()!r}')


def main(argv=None):
    args = parse_args(argv)
    workflow = None
    try:
        if args.map_prefix is not None:
            validate_new_map_prefix(args.map_prefix)
        preflight(args)
        rclpy.init()
        workflow = RealRobotWorkflow(args)
        workflow.run()
    except KeyboardInterrupt:
        print('\n[EE3033] operator shutdown requested', flush=True)
    except (WorkflowError, ValueError, OSError, EOFError,
            subprocess.SubprocessError) as exc:
        print(f'\n[EE3033] STOP: {exc}', file=sys.stderr, flush=True)
        if workflow is not None:
            workflow.record_failure(exc)
        return 1
    finally:
        if workflow is not None:
            workflow.shutdown()
        elif rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
