#!/usr/bin/env python3
"""
DDS → ROS2: Publish JointState messages

Usage:
    python joint_status_publisher.py

Test with ROS 2:
    ros2 topic echo /joint_states sensor_msgs/msg/JointState
"""

import time
import math
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.idl.builtin_interfaces.msg import Time_


def main():
    """Run DDS JointState publisher."""
    print("=== DDS JointState Publisher ===")
    print("Publishing to /joint_states")
    print("ROS 2 subscribers can receive these messages with:")
    print("  ros2 topic echo /joint_states sensor_msgs/msg/JointState")
    print("\nPress Ctrl+C to stop\n")

    node = DDSNode(
        name="joint_status_publisher",
        domain_id=30,           # Must match ROS_DOMAIN_ID
        network_interface="auto",
        allow_multicast=True
    )

    pub = node.dds_create_publisher("/joint_states", JointState_)

    joint_names = [
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"
    ]

    start_time = time.time()
    print("Publisher ready! Starting to publish messages...\n")

    try:
        while True:
            now = time.time()
            sec, nanosec = int(now), int((now - int(now)) * 1e9)
            header = Header_(stamp=Time_(sec=sec, nanosec=nanosec), frame_id="base_link")

            t = now - start_time
            position = [math.sin(t + i) for i in range(len(joint_names))]
            velocity = [math.cos(t + i) for i in range(len(joint_names))]
            effort = [0.0 for _ in range(len(joint_names))]

            msg = JointState_(
                header=header,
                name=joint_names,
                position=position,
                velocity=velocity,
                effort=effort,
            )

            pub.publish(msg)
            print(f"[Published JointState] positions={position}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping publisher...")
    finally:
        node.dds_destroy_node()
        print("Publisher stopped.")


if __name__ == "__main__":
    main()
