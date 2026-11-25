#!/usr/bin/env python3
"""Verify the actual FC message ABI and add real-flight DDS topics; never builds/flashes a board."""
import argparse
import json
from pathlib import Path
import subprocess
import yaml

REQUIRED=('VehicleOdometry','VehicleStatus','VehicleLocalPosition','VehicleLandDetected',
          'VehicleCommand','VehicleCommandAck','TimesyncStatus','EstimatorAidSource1d')
PUBLICATIONS={'vehicle_status':'VehicleStatus','vehicle_local_position':'VehicleLocalPosition',
              'vehicle_land_detected':'VehicleLandDetected','vehicle_command_ack':'VehicleCommandAck',
              'timesync_status':'TimesyncStatus','estimator_aid_src_baro_hgt':'EstimatorAidSource1d'}
SUBSCRIPTIONS={'vehicle_visual_odometry':'VehicleOdometry','vehicle_command':'VehicleCommand'}


def normalized(text):
    return [' '.join(line.split('#',1)[0].split()) for line in text.splitlines()
            if line.split('#',1)[0].strip()]


def prepare(px4, messages, verify_only=False):
    for name in REQUIRED:
        a,b=px4/'msg'/f'{name}.msg',messages/f'{name}.msg'
        if not a.exists() or normalized(a.read_text())!=normalized(b.read_text()):
            raise ValueError(f'ABI mismatch: {name}.msg. Match px4_msgs to the real FC firmware first.')
    path=px4/'src/modules/uxrce_dds_client/dds_topics.yaml'
    text=path.read_text();doc=yaml.safe_load(text)
    missing=[]
    for section, direction, topics in [('publications','out',PUBLICATIONS),('subscriptions','in',SUBSCRIPTIONS)]:
        existing={item['topic']:item['type'] for item in (doc.get(section) or [])}
        entries=[]
        for name,kind in topics.items():
            topic=f'/fmu/{direction}/{name}';typ=f'px4_msgs::msg::{kind}'
            if topic in existing:
                if existing[topic]!=typ:raise ValueError(f'Wrong type for {topic}')
            else:
                missing.append(topic);entries.append(f'  - topic: {topic}\n    type: {typ}\n')
        if entries and not verify_only:
            # Normalize an empty inline list before appending a block list.
            text=text.replace(section+': []',section+':',1)
            block='\n  # ZAV real RC / conditional baro landing\n'+'\n'.join(entries)+'\n'
            if section=='publications':
                if '\nsubscriptions:' not in text:raise ValueError('Unsupported dds_topics.yaml')
                text=text.replace('\nsubscriptions:',block+'subscriptions:',1)
            else:text+=block
    if not verify_only:path.write_text(text)
    return missing


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4-dir',required=True,type=Path)
    p.add_argument('--messages-dir',type=Path,default=Path(__file__).resolve().parents[2]/'px4_msgs/msg')
    p.add_argument('--verify-only',action='store_true')
    p.add_argument('--report',type=Path)
    a=p.parse_args()
    try:
        missing=prepare(a.px4_dir,a.messages_dir,a.verify_only)
        version=subprocess.check_output(['git','-C',str(a.px4_dir),'describe','--tags','--always'],text=True).strip()
        result=dict(abi_matches=True,source_version=version,required_firmware=[1,14,3],
                    missing_topics=missing if a.verify_only else [],added_topics=missing if not a.verify_only else [])
        if a.report:a.report.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        if a.verify_only and missing:p.exit(1,'DDS topics missing; rerun without --verify-only, then build/flash the correct board target.\n')
    except (ValueError,OSError,subprocess.CalledProcessError) as error:p.exit(1,str(error)+'\n')


if __name__=='__main__':main()
