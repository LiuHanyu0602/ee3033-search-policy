# Camera/LiDAR physical alignment tool

This is an independent, non-navigating workflow. It does not modify or invoke
the proven SLAM-search-return one-command workflow. It starts the balancing
base and LiDAR, timestamp-correct camera, deployed NCNN detector, dynamic
roll/pitch camera TF, and formal YOLO adapter. It never starts Nav2 or a search
manager and never publishes a navigation goal or `/cmd_vel`.

Place one tall, ground-contacting COCO object in an otherwise empty area. The
object must be visible to the camera and intersect the horizontal LiDAR scan.
Keep the whole YOLO bounding box inside the image. From Docker, run:

```bash
cd /home/ros2_ws/src/ee3033-search-policy
./scripts/run_camera_lidar_calibration.sh --target-class bottle
```

Replace `bottle` with the deployed COCO class. The program immediately takes
the first measurement once all inputs are ready. Each result is the median of
five fresh, synchronized detections. Adjust the camera mount, press Enter, and
repeat. Press Ctrl+C to stop all processes owned by this calibration run.

The camera side uses the same calibrated raw-pixel undistortion, bbox-centre
bearing, bbox-bottom ground intersection, timestamped dynamic camera TF, and
ground-contact assumption as the real workflow. The LiDAR side uses the same
0.12 s image/scan synchronization, bbox edge angle sector, 0.05236 rad margin,
and nearest support of at least two adjacent scan beams.

Output signs are physical adjustment directions:

- positive horizontal adjustment means turn the camera left;
- negative horizontal adjustment means turn the camera right;
- positive vertical adjustment means tilt the camera down;
- negative vertical adjustment means tilt the camera up.

The vertical angle is an approximation derived from the camera ground-range
error and the LiDAR target range. Iterate using the displayed direction and
distance difference rather than applying a large correction in one step.
