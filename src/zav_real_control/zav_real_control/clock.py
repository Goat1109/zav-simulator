"""DDS timestamps are already ROS epoch. Never subtract timesync offset twice."""
import math

EPOCH_US = 1_000_000_000_000


def validate_sample(stamp_us, now_us, last_us, max_age_s):
    if stamp_us < EPOCH_US:
        return 'source_not_ros_epoch'
    if stamp_us <= last_us:
        return 'nonmonotonic_source_stamp'
    age = (now_us-stamp_us)/1e6
    if not math.isfinite(age) or age < -.005 or age > max_age_s:
        return 'source_measurement_age'
    return None


def validate_dds_clock(now_us, status_stamp_us, status_rx, sync, sync_rx, now, c):
    if status_rx is None or now-status_rx > c['max_feedback_age_s']:
        return 'px4_status_missing_or_stale'
    if status_stamp_us < EPOCH_US or abs(now_us-status_stamp_us)/1e6 > c['max_px4_clock_skew_s']:
        return 'px4_dds_not_ros_epoch_or_clock_skew'
    if sync_rx is None or now-sync_rx > c['max_timesync_age_s']:
        return 'dds_timesync_missing_or_stale'
    if sync['source_protocol'] != 2 or not 0 < sync['round_trip_time'] <= c['max_timesync_rtt_us']:
        return 'dds_timesync_not_converged'
    return None
