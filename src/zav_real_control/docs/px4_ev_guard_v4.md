# EV guard v4 / checked serial transport

The bridge now uses `CheckedSerial` instead of pymavlink's error-swallowing
`mavserial.write`: writes must return the entire frame length. Exceptions and
short writes become persistent OSError failures visible in status. Failed writes
do not increment EV output counts. Automatic serial reconnect is disabled; recover
only with a ground-side bridge restart after checking the physical connection.

Serial writes have a 20 ms timeout (not a hard realtime scheduling guarantee).
Each receive tick handles at most 64 messages and stops between messages once
2 ms has elapsed, reducing callback starvation during incoming bursts. There is
no new thread or camera subscriber. FAST-LIVO code/tuning and the 250 ms guard
threshold are unchanged.

Commander now copies the latest EV before capturing `now`, so a concurrent
publication between the previous `now` capture and subscription update cannot
make a valid receipt appear to originate in the future. Truly future-dated sample
timestamps remain invalid; the freshness predicate/threshold is not relaxed.
This removes a race, but does not establish that the race caused log_44's failure.

Validation: 57 tests passed, 1 skipped (optional baseline-dependent fixture), ROS
package and HKUST NXT dual firmware builds succeeded. An isolated pseudo-terminal
smoke test confirmed full writes and persistent disconnect errors using pyserial.
No arming or motor commands were sent. No live flight validation was performed.

Artifact: `firmware_build/artifacts/hkust_nxt-dual_v1.17.0_zav_ev_guard_v4.px4`.
See `manifest_v4.json` for digest. v3 artifact is preserved. v4 has NOT been flashed.
The bridge changes take effect on the next bridge process restart after building;
the FC race fix requires installing v4. Use the existing backup/upload/check
procedure on the ground. After upload read parameters back; the version number
alone is insufficient to distinguish custom builds.

Battery read-back on 2026-10-30: COM_LOW_BAT_ACT=2 (Land), BAT_LOW_THR=.15,
BAT_CRIT_THR=.07, BAT_EMERGEN_THR=.05, COM_FLTT_LOW_ACT=0. No battery
parameters were changed during this work. Critical warning (estimated remaining
below 7%) selects Land; 15% is warning-only. Existing arming-state warning and
native failsafe handling still apply, so this is not an exact voltage trigger.
