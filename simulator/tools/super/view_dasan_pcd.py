#!/usr/bin/env python3
"""Show the exported coloured map in RViz without running a planner/controller."""
import argparse
from pathlib import Path
import signal
import subprocess
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from pcd_scene import read_pcd


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[2]/'assets/super')
    parser.add_argument('--config',type=Path,required=True)
    args=parser.parse_args();ws=Path(__file__).resolve().parents[3]
    xyz=read_pcd(args.assets/'dasan.pcd');rgb=np.load(args.assets/'dasan_color.npz')['rgb']
    records=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    for i,k in enumerate(('x','y','z')):records[k]=xyz[:,i]
    records['rgb']=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
    config=yaml.safe_load((ws/'src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz').read_text())
    manager=config['Visualization Manager'];display=manager['Displays'][1]['Displays'][1]
    display['Name']='Dasan exported FAST-LIVO2 map';display['Enabled']=True
    display['Topic']['Value']='/dasan/inspection_map';display['Decay Time']=0.
    display['Size (m)']=.06;display['Queue Size']=1
    manager['Displays']=[display];manager['Global Options']['Fixed Frame']='world'
    view=manager['Views']['Current'];view.update(Distance=130.,Pitch=.55,Yaw=3.95)
    view['Focal Point']={'X':30.,'Y':-5.,'Z':7.}
    config['Window Geometry']={'Width':1100,'Height':820,'X':780,'Y':80}
    args.config.parent.mkdir(parents=True,exist_ok=True);args.config.write_text(yaml.safe_dump(config,sort_keys=False))
    rclpy.init();node=Node('dasan_map_inspector')
    pub=node.create_publisher(PointCloud2,'/dasan/inspection_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg=PointCloud2();msg.header.frame_id='world';msg.height=1;msg.width=len(records)
    msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.UINT32 if k=='rgb'else PointField.FLOAT32,count=1)
                for i,k in enumerate(('x','y','z','rgb'))]
    msg.point_step=16;msg.row_step=16*len(records);msg.is_dense=True;msg.data=records.tobytes()
    def publish():
        msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg)
    node.create_timer(2.,publish)
    rviz=subprocess.Popen(['bash',str(ws/'simulator/tools/rviz/gui_exec.sh'),'rviz2','-d',str(args.config)])
    try:
        publish()
        while rclpy.ok()and rviz.poll()is None:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        if rviz.poll()is None:rviz.send_signal(signal.SIGINT);rviz.wait(timeout=20)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
