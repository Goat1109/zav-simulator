# Source this file, from any working directory. Keep the simulator workspace out of this shell.
_real_package="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_real_workspace="$(cd -- "$_real_package/../.." && pwd)"
export ZAV_REAL_CONFIG="${ZAV_REAL_CONFIG:-$_real_package/config/hardware.yaml}"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CYCLONEDDS_URI
source /opt/ros/humble/setup.bash || return 1
if [ -f "$_real_workspace/.venv/bin/activate" ]; then source "$_real_workspace/.venv/bin/activate"; fi
if [ -f "$_real_workspace/install/local_setup.bash" ]; then source "$_real_workspace/install/local_setup.bash"; fi
export ROS_DOMAIN_ID="$(python3 - "$ZAV_REAL_CONFIG" <<'PY'
import sys,yaml
with open(sys.argv[1]) as f: print(yaml.safe_load(f).get('ros_domain_id',23))
PY
)"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export PYTHONNOUSERSITE=1
unset _real_package _real_workspace
