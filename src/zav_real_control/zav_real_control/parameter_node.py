"""Real FC MAVLink parameter endpoint. No simulator/UDP-port assumptions."""
import fcntl
import json
import os
import time
from .config import load_config
from .mavlink_parameters import ParameterLink


def main(args=None):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from px4_msgs.msg import VehicleStatus
    from std_msgs.msg import Bool, String
    from pymavlink import mavutil
    rclpy.init(args=args)
    node = rclpy.create_node('zav_real_baro_link')
    lock = open(f'/tmp/zav-real-baro-{os.getuid()}.lock','w')
    connection = None
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        path = node.declare_parameter('config_file','').value
        c = load_config(path)
        publish = node.declare_parameter('publish_enabled',c['publish_enabled']).value
        allow = publish and c['calibration_verified'] and c['firmware_abi_verified']
        connection = mavutil.mavlink_connection(c['mavlink']['connection'], baud=c['mavlink']['baud'],
            source_system=c['source_system'],source_component=c['source_component'],autoreconnect=True)
        link = ParameterLink(connection,c['expected_firmware'])
        state = dict(desired=0,armed=True,stamp=None,rx=None,error=None)
        reliable = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(String,c['status_prefix']+'/baro_parameter',reliable)

        def request(msg):
            if msg.data:
                state['desired']=1  # Sticky until this process restarts on the ground.

        def status(msg):
            target = (msg.system_id,msg.component_id)
            if (0 in target or (link.target and link.target!=target)
                    or (state['stamp'] is not None and msg.timestamp<=state['stamp'])):
                return
            link.target=target
            state.update(armed=msg.arming_state==VehicleStatus.ARMING_STATE_ARMED,
                         stamp=msg.timestamp,rx=time.monotonic())

        node.create_subscription(Bool,c['status_prefix']+'/baro_enable',request,reliable)
        node.create_subscription(VehicleStatus,c['px4_prefix']+'/out/vehicle_status',status,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))

        def tick():
            now=time.monotonic()
            try:
                if state['rx'] is not None and now-state['rx'] < c['max_feedback_age_s']:
                    link.tick(state['desired'],state['armed'],now,allow)
                else:
                    link.poll(now)
                state['error']=None
            except OSError as error:
                state['error']=str(error)
                link.value_rx=link.heartbeat_rx=None
            data=link.snapshot(state['desired'],now)
            data.update(writes_enabled=bool(allow),error=state['error'])
            pub.publish(String(data=json.dumps(data)))

        node.create_timer(.1,tick)
        node.get_logger().info(f'MAVLink {c["mavlink"]["connection"]}; parameter writes={allow}; '
                               f'expected PX4={c["expected_firmware"]}')
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if connection:
            connection.close()
        lock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
