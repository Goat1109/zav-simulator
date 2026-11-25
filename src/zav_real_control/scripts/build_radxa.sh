#!/usr/bin/env bash
# Run on Dragon Q6A. Does not build/flash PX4 or launch/arm any aircraft.
set -eo pipefail
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
real_workspace="$(cd -- "$package_root/../.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source "/opt/ros/$ros_distro/setup.bash"
cd "$real_workspace"
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r "$package_root/requirements.txt"
# Existing sensor/SDK installation is kept. Optional camera/dehaze/sender packages are not selected.
rosdep install --from-paths src/zav_real_control src/FAST-LIVO2 src/rpg_vikit src/sophus src/livox_ros_driver2 \
  --ignore-src -r -y --rosdistro "$ros_distro" --skip-keys "OpenCV Eigen3"
MAKEFLAGS="-j${ZAV_BUILD_JOBS:-2}" CMAKE_BUILD_PARALLEL_LEVEL="${ZAV_BUILD_JOBS:-2}" \
.venv/bin/python "$(command -v colcon)" build --symlink-install --executor sequential \
  --packages-up-to zav_real_control fast_livo \
  --cmake-args -DROS_EDITION=ROS2 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$real_workspace/.venv/bin/python" -DPYTHON_EXECUTABLE="$real_workspace/.venv/bin/python"
