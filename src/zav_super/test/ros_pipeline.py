#!/usr/bin/env python3
"""Isolated SUPER/OMMPC integration test. Synthetic ROS inputs, no FC transport.

Run with ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 after sourcing install/setup.bash.
The plant is an ideal body-rate/collective-acceleration model, NOT PX4 or hardware.
"""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID') == '218'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mars_quadrotor_msgs.msg import PolynomialTrajectory
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def run(obstacle):
    result = {'obstacle': obstacle, 'model': 'ideal body-rate/acceleration; no FC'}
    share = Path(get_package_share_directory('zav_super'))
    config = yaml.safe_load((share/'config/common.yaml').read_text())
    config['fsm']['click_goal_topic'] = '/zav/super/goal'
    with tempfile.TemporaryDirectory(prefix='zav-super-offline-') as temp:
        temp = Path(temp)
        (temp/'common.yaml').write_text(yaml.safe_dump(config))
        controller = dict(config['argo_ommpc_px4']['ros__parameters'])
        for key in ('use_px4_landing', 'allow_terminal_hold'):
            controller.pop(key, None)
        controller.update(execute=True, distance_m=7.0, use_sim_time=False)
        (temp/'controller.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': controller}}))
        processes, handles = [], []
        node = Node('offline_super_plant')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        odom_pub = node.create_publisher(Odometry, '/argo/super/odom', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/argo/super/cloud_world', 1)
        status_pub = node.create_publisher(String, '/zav_real/status', qos)
        commands, plans = [], []
        node.create_subscription(String, '/zav/super/rates', lambda m: commands.append(json.loads(m.data)), 1)
        from rclpy.qos import qos_profile_sensor_data
        node.create_subscription(PolynomialTrajectory, '/planning_cmd/poly_traj',
                                 lambda m: plans.append(m) if m.type & m.POSITION_TRAJ else None,
                                 qos_profile_sensor_data)
        client = node.create_client(Trigger, '/zav/super/go_front')
        p, v, rotation = np.array([0., 0., 2.]), np.zeros(3), Rotation.identity()
        path = []
        # Dense registered world returns: enclosing room and optional solid box.
        points = []
        for x in np.arange(-2., 11., .25):
            for y in np.arange(-4., 4.1, .25):
                points.extend([(x, y, 0., 1.), (x, y, 5., 1.)])
        for x in np.arange(-2., 11., .25):
            for z in np.arange(0., 5.1, .25):
                points.extend([(x, -4., z, 1.), (x, 4., z, 1.)])
        for y in np.arange(-4., 4.1, .25):
            for z in np.arange(0., 5.1, .25):
                points.extend([(-2., y, z, 1.), (11., y, z, 1.)])
        if obstacle:
            for y in np.arange(-.6, .61, .10):
                for z in np.arange(.1, 3.81, .10):
                    points.extend([(3., y, z, 1.), (3.5, y, z, 1.)])
            for x in np.arange(3., 3.51, .10):
                for z in np.arange(.1, 3.81, .10):
                    points.extend([(x, -.6, z, 1.), (x, .6, z, 1.)])
        raw = b''.join(struct.pack('<ffff', *xyz) for xyz in points)
        cloud = PointCloud2(height=1, width=len(points), point_step=16,
                            row_step=16*len(points), is_dense=True, data=raw)
        cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.header.frame_id = 'world'
        last_cloud = last_status = 0.
        latest_command = None
        goal_sent = False
        try:
            for package, executable, args in (
                ('super_planner', 'fsm_node', ['-p', f'config_name:={temp}/common.yaml', '-p', 'use_sim_time:=false']),
                ('zav_super', 'super_ommpc', ['--params-file', str(temp/'controller.yaml')])):
                log = open(temp/f'{executable}.log', 'w+')
                handles.append(log)
                binary = Path(get_package_prefix(package))/'lib'/package/executable
                processes.append(subprocess.Popen([str(binary), '--ros-args', *args], stdout=log,
                                                   stderr=subprocess.STDOUT, start_new_session=True))
            begin = last = time.monotonic()
            settled_at = None
            last_index = 0
            while time.monotonic()-begin < 22.:
                now = time.monotonic()
                dt, last = min(now-last, .03), now
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.)
                if len(commands) > last_index:
                    latest_command = commands[-1]
                    last_index = len(commands)
                if goal_sent and latest_command and latest_command['valid']:
                    rotation = rotation * Rotation.from_rotvec(np.array(latest_command['rates_flu'])*dt)
                    acceleration = rotation.apply([0., 0., latest_command['acceleration']])-np.array([0., 0., 9.81])
                    p += v*dt+.5*acceleration*dt*dt
                    v += acceleration*dt
                stamp = node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'world', 'argo_body'
                odom.header.stamp = stamp
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, p)
                q = rotation.as_quat()
                odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = map(float, q)
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, v)
                odom_pub.publish(odom)
                if now-last_cloud >= .10:
                    cloud.header.stamp = stamp
                    cloud_pub.publish(cloud)
                    last_cloud = now
                if now-last_status >= .10:
                    status_pub.publish(String(data=json.dumps({'ready': True, 'latched': False,
                        'offboard': {'source': 'super', 'fc_mode': [6, 0], 'stopped': False}})))
                    last_status = now
                if not goal_sent and now-begin > 3. and client.service_is_ready() and latest_command and latest_command['valid']:
                    future = client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
                    assert future.done() and future.result().success, 'Goal rejected'
                    goal_sent = True
                    result['goal'] = [7., 0., 2.]
                if goal_sent:
                    path.append([now-begin, *p.tolist(), *v.tolist()])
                    if np.linalg.norm(p-[7., 0., 2.]) < .3 and np.linalg.norm(v) < .3:
                        settled_at = settled_at or now
                        if now-settled_at > 1.:
                            break
                    else:
                        settled_at = None
                assert all(process.poll() is None for process in processes), 'Child exited'
                if latest_command and latest_command['phase'] in ('NO_PLAN', 'PLANNER_TIMEOUT', 'SOLVER_FAILURE', 'ODOMETRY_JUMP', 'SUPER_EMERGENCY'):
                    raise AssertionError(f'Controller fault: {latest_command}')
                time.sleep(.005)
            assert goal_sent and plans and any(m['phase']=='TRACKING' for m in commands), 'No SUPER tracking'
            assert settled_at is not None, f'Goal not reached: {p}; {latest_command}'
            assert all(np.isfinite(m['rates_flu']).all() and np.isfinite(m['acceleration']) for m in commands)
            # Repeat an old cloud with fresh deliveries: measurement age, not merely
            # receipt time, must stop the controller. Odom and bridge stay fresh.
            before = len(commands)
            until = time.monotonic()+.9
            while time.monotonic() < until:
                odom.header.stamp = node.get_clock().now().to_msg()
                odom_pub.publish(odom)
                cloud_pub.publish(cloud)
                status_pub.publish(String(data=json.dumps({'ready': True, 'latched': False,
                    'offboard': {'source': 'super', 'fc_mode': [6, 0], 'stopped': False}})))
                rclpy.spin_once(node, timeout_sec=.01)
            assert any(not m['valid'] and m['phase']=='FEEDBACK_WAIT' for m in commands[before:]), 'Cloud timeout was not rejected'
            result.update(success=True, plans=len(plans), commands=len(commands),
                          final_position=p.tolist(), path=path, cloud_loss_rejected=True,
                          max_solver_ms=max(m['solver_ms'] for m in commands))
            if obstacle:
                distances = [np.linalg.norm(np.maximum(np.maximum(np.array([3., -.6, .1])-row[1:4],
                                      np.array(row[1:4])-[3.5, .6, 3.8]), 0.)) for row in path]
                result['minimum_box_clearance_m'] = float(min(distances))
                assert min(distances) > .35, 'Synthetic path intersects inflated obstacle'
            return result
        except Exception:
            debug=Path('/tmp/zav-super-offline-debug.json')
            debug.write_text(json.dumps({'path':path,'commands':commands,
                'plans':[{'start':m.start_wt_pos,'time':list(m.time_pos),'x':list(m.coef_pos_x),'y':list(m.coef_pos_y),'z':list(m.coef_pos_z)} for m in plans]}))
            for handle in handles:
                handle.flush();handle.seek(0)
                print(handle.name, handle.read()[-12000:], flush=True)
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL);process.wait()
            for handle in handles:handle.close()
            node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        results = [run(False), run(True)]
        out = Path(__file__).resolve().parents[3]/'log/hardware_checks/super_offline_pipeline.json'
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(results, indent=2))
        print(json.dumps([{k:v for k,v in r.items() if k!='path'} for r in results], indent=2))
        print(out)
    finally:rclpy.shutdown()
