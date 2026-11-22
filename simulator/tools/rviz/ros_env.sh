# [ARGO 변경] 공통 ROS 환경: Ubuntu는 Fast DDS, WSL은 검증된 Cyclone TCP를 사용합니다.
_argo_config="${ARGO_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/argo/simulation.env}"
[ ! -f "$_argo_config" ] || source "$_argo_config"
export ARGO_WS_DIR="${ARGO_WS_DIR:-$HOME/zenith_ws}"
export PX4_DIR="${PX4_DIR:-$HOME/zenith_ws/src/PX4-Autopilot}"
source /opt/ros/humble/setup.bash || return 1
_argo_rviz_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 실행 위치와 DDS 선택은 독립적입니다 (WSL 안의 Linux Isaac도 명시적으로 선택 가능).
export ARGO_WINDOWS_HOST=0
if [ "${ARGO_SIM_PLATFORM:-auto}" = wsl ] || { [ "${ARGO_SIM_PLATFORM:-auto}" = auto ] && grep -qi microsoft /proc/version; }; then
    export ARGO_WINDOWS_HOST=1
fi
_argo_transport="${ARGO_ROS_TRANSPORT:-auto}"
if [ "$_argo_transport" = auto ]; then
    if [ "$ARGO_WINDOWS_HOST" = 1 ]; then _argo_transport=cyclone_tcp; else _argo_transport=fastdds; fi
fi
case "$_argo_transport" in
    cyclone_tcp)
        # 시스템 apt 설치와 기존 사용자 로컬 설치를 모두 허용합니다.
        _argo_dds_root="$HOME/.local/share/pegasus-cyclonedds/opt/ros/humble"
        if [ -f "$_argo_dds_root/lib/librmw_cyclonedds_cpp.so" ]; then
            export AMENT_PREFIX_PATH="$_argo_dds_root:${AMENT_PREFIX_PATH:-}"
            export CMAKE_PREFIX_PATH="$_argo_dds_root:${CMAKE_PREFIX_PATH:-}"
            export LD_LIBRARY_PATH="$_argo_dds_root/lib:$_argo_dds_root/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
        elif [ ! -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
            echo 'Cyclone DDS가 필요합니다: sudo apt install ros-humble-rmw-cyclonedds-cpp' >&2
            return 1
        fi
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="file://$_argo_rviz_dir/cyclonedds_local.xml"
        ;;
    fastdds)
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        unset CYCLONEDDS_URI
        ;;
    *) echo "잘못된 ARGO_ROS_TRANSPORT: $_argo_transport (auto/fastdds/cyclone_tcp)" >&2; return 1 ;;
esac
export ARGO_ROS_TRANSPORT="$_argo_transport"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset _argo_config _argo_rviz_dir _argo_transport _argo_dds_root
