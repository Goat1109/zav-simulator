#!/usr/bin/env python3
"""Prepare MAVLink diagnostics for reviewed PX4 v1.14.3 or v1.17.0 sources.

Source preparation is not firmware build, hardware verification, or flash approval.
"""
import argparse
from pathlib import Path
import subprocess

BARO='''        // ZAV_SINGLE_LINK_BARO: primary EKF baro fusion on the same MAVLink UART.
        bool zav_instance_ok = true;
        if (_zav_baro_sub.get_instance() != _estimator_status_sub.get_instance()) {
            zav_instance_ok = _zav_baro_sub.ChangeInstance(_estimator_status_sub.get_instance());
        }
        estimator_aid_source1d_s zav_baro{};
        if (zav_instance_ok && _zav_baro_sub.update(&zav_baro)
            && zav_baro.estimator_instance == _estimator_status_sub.get_instance()
            && zav_baro.timestamp_sample > 0
            && hrt_elapsed_time(&zav_baro.timestamp_sample) < 500000) {
            const bool good = zav_baro.fused && zav_baro.time_last_fuse > 0
                && !zav_baro.innovation_rejected && PX4_ISFINITE(zav_baro.test_ratio)
                && zav_baro.test_ratio >= 0.f && zav_baro.test_ratio <= 1.f;
            const int bits = (zav_baro.fusion_enabled ? 1 : 0) | (good ? 2 : 0)
                | (zav_baro.innovation_rejected ? 4 : 0);
            const char name[10] = "zav_baro";
            mavlink_msg_named_value_float_send(_mavlink->get_channel(),
                hrt_absolute_time() / 1000, name, static_cast<float>(bits));
        }

'''
SYNC='''
            // ZAV_SINGLE_LINK_SYNC: report the FC filter's actual convergence, not an SBC guess.
            const char name[10] = "zav_sync";
            mavlink_msg_named_value_float_send(_mavlink->get_channel(), now / 1000,
                name, _timesync.sync_converged() ? 1.f : 0.f);
'''
SYNC_117 = SYNC.replace('_mavlink->get_channel()', '_mavlink.get_channel()')

ADVERTISE_117 = '''
#if defined(CONFIG_EKF2_BAROMETER)
    // ZAV_SINGLE_LINK_BARO_INSTANCE: reserve in constructor order even with BARO_CTRL=0.
    // Deferred advertisement can assign aid-source topics to a different EKF index.
    _estimator_aid_src_baro_hgt_pub.advertise();
#endif // CONFIG_EKF2_BAROMETER
'''

# PX4 1.17 removed fusion_enabled from EstimatorAidSource1d. Read the
# primary estimator's control flags separately; never infer fusion from PARAM_VALUE.
BARO_117='''        // ZAV_SINGLE_LINK_BARO: primary EKF baro diagnostics for PX4 v1.17.0.
        const auto zav_primary = _estimator_status_sub.get_instance();
        estimator_selector_status_s zav_selector{};
        bool zav_instance_ok = !_estimator_selector_status_sub.advertised()
            || (_estimator_selector_status_sub.copy(&zav_selector)
                && zav_selector.primary_instance == zav_primary);
        if (_zav_baro_sub.get_instance() != zav_primary) {
            zav_instance_ok = _zav_baro_sub.ChangeInstance(zav_primary) && zav_instance_ok;
        }
        if (_zav_flags_sub.get_instance() != zav_primary) {
            zav_instance_ok = _zav_flags_sub.ChangeInstance(zav_primary) && zav_instance_ok;
        }
        estimator_aid_source1d_s zav_baro{};
        estimator_status_flags_s zav_flags{};
        if (zav_instance_ok && _zav_baro_sub.update(&zav_baro)
            && _zav_flags_sub.copy(&zav_flags)
            && zav_baro.estimator_instance == zav_primary
            && zav_baro.timestamp_sample > 0
            && hrt_elapsed_time(&zav_baro.timestamp_sample) < 500000
            // Status flags publish at 1 Hz, and immediately on status changes.
            && zav_flags.timestamp > 0
            && hrt_elapsed_time(&zav_flags.timestamp) < 2000000) {
            const bool good = zav_flags.cs_baro_hgt && zav_baro.fused
                && zav_baro.time_last_fuse > 0
                && hrt_elapsed_time(&zav_baro.time_last_fuse) < 500000
                && !zav_baro.innovation_rejected && PX4_ISFINITE(zav_baro.test_ratio)
                && zav_baro.test_ratio >= 0.f && zav_baro.test_ratio <= 1.f;
            const int bits = (zav_flags.cs_baro_hgt ? 1 : 0) | (good ? 2 : 0)
                | (zav_baro.innovation_rejected ? 4 : 0);
            const char name[10] = "zav_baro";
            mavlink_msg_named_value_float_send(_mavlink->get_channel(),
                hrt_absolute_time() / 1000, name, static_cast<float>(bits));
        }

'''


def edits(px4, baseline='v1.14.3'):
    if baseline not in ('v1.14.3', 'v1.17.0'):
        raise ValueError(f'Unsupported baseline: {baseline}')
    modern = baseline == 'v1.17.0'
    extra = {}
    if modern:
        aid = (px4/'msg/EstimatorAidSource1d.msg').read_text()
        flags = (px4/'msg/EstimatorStatusFlags.msg').read_text()
        timesync = (px4/'src/lib/timesync/Timesync.hpp').read_text()
        if ('fusion_enabled' in aid or 'bool cs_baro_hgt' not in flags
                or 'bool sync_converged() const' not in timesync):
            raise ValueError('Unsupported PX4 v1.17.0 diagnostic message/API schema')
        ekf_path = px4/'src/modules/ekf2/EKF2.cpp'
        ekf = ekf_path.read_text()
        anchor = '\t_estimator_status_flags_pub.advertise();'
        if 'ZAV_SINGLE_LINK_BARO_INSTANCE' not in ekf:
            if ekf.count(anchor) != 1:
                raise ValueError('Unsupported PX4 v1.17.0 EKF topic advertisement source')
            ekf = ekf.replace(anchor, anchor+ADVERTISE_117)
        elif ADVERTISE_117 not in ekf:
            raise ValueError('Existing baro instance patch differs from reviewed patch')
        extra[ekf_path] = ekf
    stream=px4/'src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp'
    sync=px4/'src/modules/mavlink/mavlink_timesync.cpp'
    a,b=stream.read_text(),sync.read_text()
    if 'ZAV_SINGLE_LINK_BARO' not in a:
        required=['#include <uORB/topics/estimator_status.h>',
                  '\tuORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};',
                  '\t\testimator_status_s est;',
                  'MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES']
        if any(a.count(anchor)!=1 for anchor in required):
            raise ValueError(f'Unsupported ESTIMATOR_STATUS source for {baseline}')
        a=a.replace(required[0],required[0]+'\n#include <uORB/topics/estimator_aid_source1d.h>\n#include <drivers/drv_hrt.h>')
        a=a.replace(required[1],required[1]+'\n\tuORB::Subscription _zav_baro_sub{ORB_ID(estimator_aid_src_baro_hgt)};')
        if modern:
            a=a.replace('#include <uORB/topics/estimator_status.h>',
                        '#include <uORB/topics/estimator_status.h>\n#include <uORB/topics/estimator_status_flags.h>')
            a=a.replace('\tuORB::Subscription _zav_baro_sub{ORB_ID(estimator_aid_src_baro_hgt)};',
                        '\tuORB::Subscription _zav_baro_sub{ORB_ID(estimator_aid_src_baro_hgt)};\n\tuORB::Subscription _zav_flags_sub{ORB_ID(estimator_status_flags)};')
        a=a.replace(required[2],(BARO_117 if modern else BARO)+required[2])
        a=a.replace(required[3],required[3]+' + MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES')
    elif modern and BARO_117 not in a:
        raise ValueError('Existing baro patch differs from the reviewed v1.17.0 patch')
    if 'ZAV_SINGLE_LINK_SYNC' not in b:
        anchor='\t\t\t\t_timesync.update(now, tsync.tc1, tsync.ts1);'
        if b.count(anchor)!=1:raise ValueError(f'Unsupported timesync source for {baseline}')
        b=b.replace(anchor,anchor+(SYNC_117 if modern else SYNC))
    elif modern and SYNC_117 not in b:
        raise ValueError('Existing timesync patch differs from the reviewed v1.17.0 patch')
    return {stream:a,sync:b,**extra}


def prepare(px4,verify_only=False,baseline='v1.14.3'):
    proposed=edits(px4,baseline)  # Validate both files before writing either.
    changed=[p for p,s in proposed.items() if p.read_text()!=s]
    if not verify_only:
        for p in changed:p.write_text(proposed[p])
    return changed


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4-dir',required=True,type=Path)
    p.add_argument('--verify-only',action='store_true')
    p.add_argument('--baseline',choices=('v1.14.3','v1.17.0'),default='v1.14.3')
    a=p.parse_args()
    try:
        tag=subprocess.check_output(['git','-C',str(a.px4_dir),'describe','--tags','--exact-match','HEAD'],text=True).strip()
        if tag!=a.baseline:raise ValueError(f'Expected exact {a.baseline} baseline, got {tag}; review the actual firmware first.')
        changed=prepare(a.px4_dir,a.verify_only,a.baseline)
        if a.verify_only and changed:
            p.exit(1,'MAVLink diagnostics missing. Run without --verify-only, then build/flash the correct NXT target.\n')
        print('Verified diagnostics' if a.verify_only else f'Prepared {len(changed)} source files. Build/flash the correct NXT target yourself.')
    except (OSError,ValueError,subprocess.CalledProcessError) as error:p.exit(1,str(error)+'\n')


if __name__=='__main__':main()
