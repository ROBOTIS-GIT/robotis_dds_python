#!/usr/bin/env python3
"""
DDS → ROS2: Subscribe to /joint_trajectory (trajectory_msgs/msg/JointTrajectory)

Usage:
    python joint_trajectory_subscriber.py

Test with ROS 2:
    ros2 topic pub /joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
      joint_names: ['joint_1', 'joint_2', 'joint_3'], \
      points: [{positions: [0.1, 0.2, 0.3], velocities: [0.0, 0.0, 0.0], \
                accelerations: [0.0, 0.0, 0.0], effort: [0.0, 0.0, 0.0], \
                time_from_start: {sec: 1, nanosec: 0}}]}"
"""

import time
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_


def main():
    print("=== DDS JointTrajectory Subscriber ===")
    print("Listening to /joint_trajectory from ROS 2")
    print("Test with ROS 2:")
    print("  ros2 topic pub /joint_trajectory trajectory_msgs/msg/JointTrajectory \"{...}\"")
    print("\nPress Ctrl+C to stop\n")

    node = DDSNode(
        name="joint_trajectory_subscriber",
        domain_id=30,
        network_interface="auto",
        allow_multicast=True
    )

    def callback(msg: JointTrajectory_):
        print("[Received JointTrajectory]")
        print(f"  frame_id: {msg.header.frame_id}")
        print(f"  joint_names: {msg.joint_names}")
        if msg.points:
            point = msg.points[0]
            print(f"  positions: {point.positions}")
            print(f"  velocities: {point.velocities}")
            print(f"  accelerations: {point.accelerations}")
            print(f"  effort: {point.effort}")
            print(f"  time_from_start: {point.time_from_start.sec}.{point.time_from_start.nanosec:09d}")
        print("-" * 60)

    node.dds_create_subscription("/joint_trajectory", JointTrajectory_, callback)

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
