# Timestamped LiDAR–visual localization update

This update affects only target ranging after target confirmation. It does not
change the tested detection-to-stop trigger. The real-robot overlay may keep
`yolo_detection_adapter.required_frames: 1`.

## Current pipeline

For every fresh post-confirmation `TargetObservation`, the stamped localizer:

1. uses the original image timestamp and dynamic camera TF to project the bbox
   bottom-centre ray onto the ground plane when ground contact is enabled;
2. selects the closest buffered `/scan` by source timestamp, with a default
   maximum image/scan difference of `0.12 s`;
3. transforms the visual centre ray into the scan frame and applies the legacy
   LiDAR angular-window median/MAD filter;
4. transforms the resulting LiDAR point at the scan timestamp into `map`;
5. accepts LiDAR preferentially when its map point agrees with visual ground
   projection within `0.35 m`, otherwise rejects the likely wall return and
   uses the visual point; and
6. robustly fuses repeated points and retains bearing-only RANSAC when enough
   camera-origin baseline is naturally available.

The default localizer now waits for five valid range observations. The mission
coordinator still requires three successive consistent candidates, so the
first report normally needs about seven distinct post-stop source frames. At
the deployed 5–6 Hz detector rate this is roughly 1.2–1.5 seconds.

The output `method` identifies the path used:

- `lidar_visual_range_fusion`: LiDAR and visual ground projection agreed;
- `ground_projection_fusion_lidar_rejected`: a LiDAR return was present but
  disagreed with vision and was rejected;
- `ground_projection_fusion`: no usable matched LiDAR return was available;
- `lidar_range_fusion`: LiDAR was used without a visual ground point;
- `multiview_ransac_fusion`: sufficient natural bearing baseline was available.

## Physical accuracy trial

Keep the target fixed on the ground and record at least five measured map-frame
ground-truth positions at near, middle and far ranges. For each placement,
record `/target/estimate`, its `method`, the LiDAR/visual status, and Euclidean
error. Do not combine trials from different target classes or elevated targets.

The parameters most worth tuning from recorded data are:

- `scan_sync_tolerance_s`: reduce only if synchronized data remain available;
- `scan_window_rad`: narrow windows reduce adjacent-wall contamination;
- `lidar_ground_gate_m`: set from observed LiDAR-versus-vision residuals;
- `minimum_range_observations`: 5 is the initial accuracy/latency compromise;
- `minimum_report_frames` and `report_consistency_m`: final candidate stability.

Do not tune the motion-interrupt thresholds to ordinary PID balancing jitter.
Large-motion resets continue to clear both range history and pending reports.

Offline validation:

```bash
python3 -m compileall -q ee3033_search_policy launch test
python3 -m pytest test -q
```

