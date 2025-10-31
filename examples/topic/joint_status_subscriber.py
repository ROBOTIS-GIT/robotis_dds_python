#!/usr/bin/env python3
"""
DDS → ROS2: Subscribe to /joint_states (sensor_msgs/JointState)

Usage:
    python joint_status_subscriber.py

Test with ROS 2:
    ros2 topic pub /joint_states sensor_msgs/msg/JointState \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], \
    position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], \
    velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
    effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
"""

import time
from robotis_dds_python.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.tools.dds_node import DDSNode


def main():
    print("=== DDS JointState Subscriber ===")
    print("Listening to /joint_states from ROS 2")
    print("Test with ROS 2:")
    print("  ros2 topic pub /joint_states sensor_msgs/msg/JointState \"{...}\"")
    print("\nPress Ctrl+C to stop\n")

    node = DDSNode(
        name="joint_status_subscriber",
        domain_id=30,
        network_interface="auto",
        allow_multicast=True
    )

    def callback(msg: JointState_):
        print(f"[Received JointState]")
        print(f"  time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        print(f"  names: {msg.name}")
        print(f"  positions: {msg.position}")
        print(f"  velocities: {msg.velocity}")
        print(f"  efforts: {msg.effort}")
        print("-" * 60)

    node.dds_create_subscription("/joint_states", JointState_, callback)

    print("Subscriber ready! Waiting for messages...\n")

    try:
        node.dds_spin()
    except KeyboardInterrupt:
        print("\nStopping subscriber...")
    finally:
        node.dds_destroy_node()
        print("Subscriber stopped.")


if __name__ == "__main__":
    main()
