# Motion-interrupt target localization handoff

## Behaviour implemented

The target mission no longer waits for a fixed-duration odometry stability
window after YOLO confirmation.  A confirmed, fresh observation now causes:

1. immediate transition to `LOCALIZING_TARGET`;
2. asynchronous navigation cancellation through the existing stop token;
3. immediate accumulation of stamped localization candidates; and
4. final publication only after the same stop token has a fresh, remotely
   terminal Nav2 acknowledgement.

Normal two-wheel balancing jitter is not evaluated against RMS or peak velocity
limits.  Odometry is now a large-motion interrupt monitor.  If translation or
yaw change exceeds the configured limit over the rolling horizon, accumulated
coordinates are cleared in both the coordinator and stamped localizer.  The
mission remains in `LOCALIZING_TARGET` and can rebuild from fresh frames.

## Safety boundary

`/target/estimate` and `/target_position` remain blocked unless all of these are
true at publication time:

- the configured episode, class and timestamp are current;
- calibrated dynamic camera TF handling is verified;
- odometry input is fresh, monotonic and in the expected frame;
- no large-motion interrupt invalidated the current fusion history;
- the coordinate estimate passes covariance and multi-frame consistency checks;
- the navigation guard confirms a remote terminal result for the current stop
  token (a local cancel request or timeout is insufficient).

This monitor is not an emergency stop and does not prove mechanical stillness.

## New runtime interface

- `/target/localization_reset` (`std_msgs/msg/String`, transient local): JSON
  containing `episode_id`, unique `token`, `generation`, `reason`,
  `not_before`, and publication `stamp`.
- `motion_interrupt_verified` must remain `false` until robot validation.
- `motion_interrupt_horizon_s`
- `motion_interrupt_max_age_s`
- `motion_interrupt_max_gap_s`
- `motion_interrupt_max_translation_m`
- `motion_interrupt_max_rotation_rad`

The checked-in defaults of 0.15 m and 0.35 rad are provisional safety values,
not measured acceptance results.

## Robot validation still required

Run with navigation disabled first and record synchronized `/odom`,
`/target/observation`, `/target/estimate_candidate`, `/target/estimate`,
`/target/localization_reset`, `/search/runtime_state`, and `/mission/state`.
Verify:

1. ordinary balancing does not publish `ODOMETRY_MOTION_INTERRUPT`;
2. deliberately moving the robot more than the configured bound publishes one
   reset and prevents all pre-movement candidates from becoming final;
3. localization starts immediately after YOLO confirmation;
4. no final coordinate is published before matching Nav2 terminal confirmation;
5. after a motion reset, only newer image-stamped observations rebuild the
   coordinate estimate;
6. stale odometry, TF failure, camera mismatch, unknown Nav2 terminal state and
   clock rewind all fail closed.

Only after these checks may `motion_interrupt_verified` be set true in a local
robot-specific parameter overlay.  Do not change the repository default merely
to make a test advance.
