#!/usr/bin/env python3
"""
DDS → ROS2: Publish JointTrajectory messages

Usage:
    python joint_trajectory_publisher.py

Test with ROS 2:
    ros2 topic echo /joint_trajectory trajectory_msgs/msg/JointTrajectory
"""

import time
import math
from robotis_dds_python.tools.dds_node import DDSNode
from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_, JointTrajectoryPoint_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.idl.builtin_interfaces.msg import Time_


def main():
    """Run DDS JointTrajectory publisher."""
    print("=== DDS JointTrajectory Publisher ===")
    print("Publishing to /joint_trajectory (trajectory_msgs/msg/JointTrajectory)")
    print("ROS 2 subscribers can receive these messages with:")
    print("  ros2 topic echo /joint_trajectory trajectory_msgs/msg/JointTrajectory")
    print("\nPress Ctrl+C to stop\n")

    node = DDSNode(
        name="joint_trajectory_publisher",
        domain_id=30,           # Must match ROS_DOMAIN_ID
        network_interface="auto",
        allow_multicast=True
    )

    pub = node.dds_create_publisher("/joint_trajectory", JointTrajectory_)

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
            point = JointTrajectoryPoint_(
                positions=[math.sin(t + i) for i in range(6)],
                velocities=[math.cos(t + i) for i in range(6)],
                accelerations=[0.0 for _ in range(6)],
                effort=[0.0 for _ in range(6)],
                time_from_start=Time_(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            )

            msg = JointTrajectory_(
                header=header,
                joint_names=joint_names,
                points=[point]
            )

            pub.publish(msg)
            print(f"[Published JointTrajectory] positions={point.positions}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping publisher...")
    finally:
        node.dds_destroy_node()
        print("Publisher stopped.")


if __name__ == "__main__":
    main()
