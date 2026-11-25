# PX4 1.17.0 FC-local EV loss guard (v3)

Target: `hkust_nxt-dual_default`, board ID 1013, upstream tag/hash
`v1.17.0 / d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
Includes the v2 timesync and primary-EKF barometer diagnostics.
This is a new firmware image; building it does not install it on the FC.

## Parameters and activation

- `ZAV_EV_GUARD=0` by default: opt-in, no automatic deployment/activation.
- `ZAV_EV_GUARD=1`: enable before arming, after bench verification.
- `ZAV_EV_TOUT=0.25` seconds by default (allowed range 0.1–2.0).
- The existing 3-second barometer handoff timeout is fixed in the policy.

The intended configuration is this vehicle's `EKF2_EV_CTRL=15` (position,
velocity, height and yaw). The FC must observe all four EV fusion flags,
a fresh fused/non-rejected primary-EKF EV height aid, valid local position
and velocity, no dead reckoning, and fresh valid EV odometry while armed.
Only then does monitoring start. No EV ever received or never healthy is
not a watchdog trigger; normal PX4 arming/mode checks remain responsible.

## Failure and recovery

Commander checks both FC receipt time and FC-timesync-adjusted measurement
age, including finite pose/velocity and valid orientation/frame checks.
No callback is required to detect a missing stream. After monitoring starts,
loss of fresh valid EV beyond the timeout latches the failure.

On latch:

1. Each EKF2 instance stops accepting external vision samples. Already buffered
   samples age out using the existing EKF logic; new data cannot reset the latch.
2. Commander enables `EKF2_BARO_CTRL=1`, retrying at most twice per second.
3. Confirmation requires parameter read-back, the selected primary EKF's fresh
   barometer flag and fused/non-rejected barometer aid, plus valid vertical
   position/velocity, with feedback timestamps after the failure.
4. On confirmation, or after 3 seconds without confirmation, a Land action enters
   PX4's native failsafe arbitration. Timeout is reported separately and does not
   claim successful barometer handoff. Native mode requirements remain in force:
   unavailable horizontal position can lead to Descend instead of Land, and
   further losses follow upstream fallback rules. No horizontal hold is guaranteed.
5. A fresh physical RC mode-slot change to Manual/Altitude/Stabilized suppresses
   this guard's Land action for the remainder of the latch. A native automatic mode
   fallback without a physical switch change does not count as pilot takeover.
   Stick movement alone does not cancel this guard and return to bad EV Position.
   Kill and other native failsafes remain intact.

The latch survives disarming and parameter disable in this boot. Commander
rejects rearming while latched, including attempts that skip normal preflight
checks. Reboot the FC on the ground to reset it. Restore and verify
`EKF2_BARO_CTRL=0` on the ground before restarting the normal EV-only operation;
the emergency enable uses the regular persistent PX4 parameter interface.
The parameter cannot silently disable monitoring after it has started while armed.

## Inspection

Use `listener zav_ev_guard_status -n 1` in MAVLink Console to inspect enabled,
watching, latched, primary instance, last EV receipt/sample timestamps,
barometer confirmation/timeout, pilot takeover and Land request.
The status is published/logged at 10 Hz in ULog.
Actual navigation mode is still reported by `vehicle_status`, independently
of the guard's request. Existing `zav_baro` telemetry remains primary-aware.

## Verification and limits

The real `EvLossGuard.hpp` is compiled and executed by
`test/test_ev_loss_guard_cpp.py`. Scenarios include disarmed/disabled/no-fusion
non-activation, missing data, timestamp boundaries, recovery without unlatching,
confirmed handoff, timeout without false confirmation, Kill, physical pilot
takeover, disarm and session reinitialization before any failure.
The board build verifies integration with Commander, native failsafe, EKF2,
uORB message generation and the logger. This is not a SITL, HIL or flight result.

The v3 image must be uploaded separately, identity/parameters checked, and the
new feature tested in simulation and with propellers removed before flight.
Do not feed synthetic flight odometry or synthetic armed messages to a real FC.
No motor/arming command is needed to inspect the default-disabled status.
Never infer a safe landing solely from `request_land=true` or `baro_confirmed=true`.

## Recorded hardware ground results (2026-10-29)

- `log/hardware_checks/20261029T101747Z_post_flash_v3_readonly.json`:
  v3 parameters/status present; all 949 prior parameters received. Changes against
  the old backup were the intentional `NAV_RCL_ACT=3` and flight UUID increment.
- `20261029T101919Z_v3_guard_enable_disarmed.json` and
  `20261029T102214Z_v3_disarmed_ev_check.json`: enabled, EV fused, disarmed guard
  remained idle, no Land request or barometer activation.
- `20261029T102819Z_v3_fc_guard_armed_loss.json`: incomplete first armed trial;
  SBC watchdog also reacted before planned isolation. Disarmed and restored baro OFF.
- `20261029T103140Z_v3_fc_guard_armed_loss.json`: corrected isolated FC trial.
  Healthy armed monitoring was observed before deliberately stopping SBC EV output.
  FC latched with the last measurement age 0.25735 s, confirmed barometer fusion,
  and entered actual AUTO.LAND (`nav_state=18`, armed, failsafe true). Recorded
  Land transition was 0.261272 s after the failure timestamp. No SBC Land or
  barometer-ON command was sent. Normal disarm and barometer-OFF restoration
  were confirmed. This was on the ground with all propellers removed; flight
  landing and later horizontal-position-loss/Descend behavior were not tested.
- `20261029T103505Z_v3_guard_reboot_check.json`: subsequent FC reboot cleared
  the latch and retained guard=1, timeout=0.25 and barometer=0; Kill ON/disarmed.
