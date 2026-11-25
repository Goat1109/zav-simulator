#!/usr/bin/env bash
# LEGACY DDS ONLY: not needed for the current single-USB-TTL MAVLink profile.
# Source build for ARM64; isolated prefix, no sudo install and no global ldconfig.
set -eo pipefail
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
real_workspace="$(cd -- "$package_root/../.." && pwd)"
agent_source="$real_workspace/.deps/Micro-XRCE-DDS-Agent"
agent_prefix="$real_workspace/.local/agent"
mkdir -p "$real_workspace/.deps"
if [ ! -d "$agent_source/.git" ]; then
  git clone --branch v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_source"
fi
[ "$(git -C "$agent_source" describe --tags --exact-match)" = v2.4.3 ] || { echo 'Agent must be v2.4.3 for this PX4 v1.14 profile.' >&2; exit 1; }
cmake -S "$agent_source" -B "$agent_source/build" -DUAGENT_SUPERBUILD=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$agent_prefix" \
  -DCMAKE_INSTALL_RPATH="$agent_prefix/lib" -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
cmake --build "$agent_source/build" --parallel "${ZAV_BUILD_JOBS:-2}"
cmake --install "$agent_source/build"
echo "Agent installed at $agent_prefix; use scripts/run_agent.sh"
