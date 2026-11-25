# robot_bundle_sender_camlidar

Robot-side camera+LiDAR bundle sender for an SBC (renamed from robot_submapper_camlidar on 2026-10-21: it does not build
submaps). The robot does no semantic computation: it collects synchronised (image, point cloud, odometry) frames and, every
`bundle_distance` metres, sends one bundle to `server_bundle_receiver`,
which unpacks it into the per-frame session layout so `semantic_submapper` runs unchanged on the server.

```
robot (C++, ROS 2)                                         server (server_bundle_receiver, Python)
image + point cloud + odometry (ApproximateTime)           gRPC CamLidarBundleService (port 50053)
  -> points per frame: voxel filter, int16 cm         -->    -> decode video, undistort, JPEG
  -> image -> ffmpeg (H.264, one segment per submap)          -> <data_dir>/<session>/images/, pcl/, poses.jsonl,
  -> per-frame stamp + pose                                     committed.jsonl, session.json, session_state.json
  -> bundle every bundle_distance m, gRPC (retries)        -> semantic_submapper (continuous_cli / lidar_anchor_cli)
```

Bundle (`proto/camlidar_bundle.proto`): per-frame points back to back (temporal geometry, LiDAR frame, zlib), one fragmented
MP4 H.264 segment with one video frame per package frame (no B-frames, starts with a key frame, decodes on its own), and per
frame the stamp, pose and point count. Calibration travels in every package.

## Build

```bash
cd ~/comm_ws && colcon build --packages-select robot_bundle_sender_camlidar && source install/setup.bash
```

Needs `ffmpeg` on the robot (software libx264 by default; set `video_codec` to a hardware encoder if the board's ffmpeg has one).

## Run

```bash
# server
cd ~/comm_ws/src/server_bundle_receiver && python -m server_bundle_receiver.cli --data-dir /path/to/root
# robot
ros2 launch robot_bundle_sender_camlidar robot_bundle_sender_camlidar.launch.py params_file:=/path/to/params.yaml
```

Stop the node with Ctrl-C: it sends the last partial bundle marked `last_in_session`, and the server sets the session CLOSED.
Parameters: `config/robot_bundle_sender_camlidar.yaml`. Keep `bundle_distance` equal to the server-side
`submap_distance` so one bundle becomes one submap.

## Live odometry stream

Besides the bundles, every message of the synchronised odometry topic (`pose_topic`) is streamed to the server as it arrives
(`StreamOdom`, own gRPC connection), independent of image/cloud synchronisation and `send_every_n_frames`. The server appends
it to `<session>/odom_live.jsonl` (`poses.jsonl` stays the bundle-unpacked per-frame poses). Optional parameters (defaults
apply when a config does not mention them): `odom_stream` (true), `odom_stream_hz` (0 = every message), `odom_stream_queue`
(200 samples kept while the server is unreachable; a failed write puts its samples back, the oldest are dropped on overflow).
The rate is bounded by the odometry topic's own rate.

## Point codec

`point_codec: zlib` (default) compresses the whole bundle's int16 points with zlib. `point_codec: draco` encodes every frame as
a Draco point cloud on an explicit one-unit (1 cm) grid, on `codec_threads` worker threads: decoded points are bit-identical to
the int16 values (point order inside a frame may differ). Needs `libdraco-dev` at build time (CMake prints whether it was
found; without it the node still builds and refuses only `draco`) and `DracoPy==2.2.0` in the receiver's Python.
Measured 2026-10-22 (per-frame 0.1 m filter, desktop core): 1.6x smaller than zlib on Mid-360, 1.8x GRACO, 2.0x NCLT
(HDL-32E), 2.3x Boreas (128-beam); encode 0.3 / 1.3 / 5 / 37 ms per frame and scales almost linearly with threads.
`draco_speed` 10 (sequential encoder) is only 1.1x better than zlib; 0..9 differ by a few percent. End to end on bungeoubang
(2,141 frames): points 8.9 MB vs 14.5 MB with zlib, every frame's point set bit-identical.

## Check (2026-10-21, handheld_bungeoubang_261015_full, 2,141 frames, replayed as ROS topics)

- Server output vs the original mapping_data_receiver session: same frames in the same order, poses and per-frame points
  bit-identical, same session.json; submap geometry (`lidar_anchor_cli`) bit-identical, anchor cells/support identical
  (anchor features differ only by Utonia's run-to-run GPU noise, the same as two runs on the original).
- Semantic (`continuous_cli`, downsample 0.2) vs the original JPEG q80 images at 1 Mbps video: feature cosine 0.960,
  language-query relevancy r 0.948 (same as the offline video test).
- Bundle: ~5 MB per 40 m (video ~3.2 MB + points ~1.7 MB); robot node 73 MB RSS, ~4 % of one desktop core incl. ffmpeg;
  server unpack ~0.6 s per bundle.
- Server undistortion = mapping_data_sender's (same intrinsics, identical pixels; fisheye and plumb_bob).
