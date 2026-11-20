#!/usr/bin/env bash
# EPIC accepts a trigger only in WAIT_TRIGGER. Wait for its state transition
# so another subscriber (such as rosbag) cannot consume the sole startup signal.
set -eu
ros2 topic echo --once --filter "m.text == 'WAIT_TRIGGER'" /epic/state visualization_msgs/msg/Marker >/dev/null
ros2 topic echo --once --filter "m.data == 'READY'" /zav/super/controller_status std_msgs/msg/String >/dev/null
ros2 topic pub -r 2 /epic/trigger nav_msgs/msg/Path \
    '{header: {frame_id: world}, poses: [{pose: {orientation: {w: 1.0}}}]}' >/dev/null &
sender=$!
trap 'kill "$sender" 2>/dev/null || true; wait "$sender" 2>/dev/null || true' EXIT
ros2 topic echo --once --filter "m.text not in ('INIT', 'WAIT_TRIGGER')" /epic/state visualization_msgs/msg/Marker >/dev/null
echo "EPIC exploration started"
