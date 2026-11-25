#!/usr/bin/env python3
"""Measure image reception using Reliable QoS (including ROS 2 Humble)."""
import argparse
from collections import deque
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('topic', nargs='?', default='/camera/image_raw')
    parser.add_argument('--duration', type=float, default=0,
                        help='Stop after this many seconds; 0 runs until Ctrl+C')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('reliable_image_hz')
    arrivals = deque(maxlen=101)
    total = 0

    def callback(msg):
        nonlocal total
        arrivals.append(time.monotonic())
        total += 1

    node.create_subscription(
        Image, args.topic, callback,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    start = last_report = time.monotonic()
    print(f'{args.topic}: Reliable QoS; Ctrl+C to stop', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_report >= 2:
                recent = [t for t in arrivals if now - t <= 10]
                if len(recent) >= 2:
                    hz = (len(recent) - 1) / (recent[-1] - recent[0])
                    gap = max(b - a for a, b in zip(recent, recent[1:]))
                    print(f'average rate: {hz:.2f} Hz | max gap: {gap:.3f}s'
                          f' | last received: {now-arrivals[-1]:.2f}s ago'
                          f' | total: {total}', flush=True)
                else:
                    print(f'Waiting for images (total received: {total})', flush=True)
                last_report = now
            if args.duration > 0 and now - start >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
