#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Robotis DDS Python SDK
# High-level wrapper for DDS-based robot communication
#
# Author: Heewon Lee, Dongyun Kim
# License: Apache 2.0

import threading
import time
import numpy as np
import cv2

# DDS Core imports
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode

# Message imports
from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import (
    CompressedImage_,
    Image_,
    JointState_,
    BatteryState_,
)
from robotis_dds_python.robotis_dds_core.idl.nav_msgs.msg import Odometry_
from robotis_dds_python.robotis_dds_core.idl.geometry_msgs.msg import (Twist_, Vector3_)
from robotis_dds_python.robotis_dds_core.idl.trajectory_msgs.msg import (
    JointTrajectory_,
    JointTrajectoryPoint_,
)
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_


class RobotisDDSSDK:
    """
    Robotis DDS Python SDK
    ----------------------
    High-level wrapper around CycloneDDS for ROS2-compatible robot data exchange.
    """

    def __init__(self, domain_id=30):
        """Initialize DDS-based Robotis SDK."""
        self.node = DDSNode(
            name="robotis_sdk_node",
            domain_id=domain_id,
            network_interface="auto",
            allow_multicast=True,
        )

        # === Internal caches ===
        self.cache = {}
        self._subscribed = {}

        # Topic mapping (topic_name → (msg_type, callback))
        self.topic_map = {
            "/camera/image": (Image_, self._image_callback),               # Image (numpy conversion)
            "/camera/image/compressed": (CompressedImage_, self._compressed_image_callback),  # Compressed Image (as is)
            "/odom": (Odometry_, self._odom_callback),
            "/joint_states": (JointState_, self._joint_state_callback),
            "/battery_state": (BatteryState_, self._battery_callback),
        }

        # === Publishers ===
        self.cmd_vel_pub = self.node.dds_create_publisher("/cmd_vel", Twist_)
        self.joint_traj_pub = self.node.dds_create_publisher("/joint_trajectory", JointTrajectory_)

        # === Spin thread ===
        self.spin_thread = threading.Thread(target=self.node.dds_spin, daemon=True)
        self.spin_thread.start()

    # ----------------------------------------------------------------------
    # Lazy subscription helper
    # ----------------------------------------------------------------------
    def _ensure_subscription(self, topic_name: str):
        """Ensure topic is subscribed (lazy init)."""
        if topic_name in self._subscribed:
            return
        if topic_name not in self.topic_map:
            print(f"[RobotisDDSSDK] ⚠ Unknown topic: {topic_name}")
            return
        msg_type, cb = self.topic_map[topic_name]
        self.node.dds_create_subscription(topic_name, msg_type, cb)
        self._subscribed[topic_name] = True
        print(f"[RobotisDDSSDK] ✅ Subscribed to {topic_name}")

    # ----------------------------------------------------------------------
    # DDS Callbacks (update internal cache)
    # ----------------------------------------------------------------------
    def _compressed_image_callback(self, msg: CompressedImage_):
        """Handle /camera/image/compressed (sensor_msgs/CompressedImage) → numpy array"""
        try:
            # ① Extract JPEG data from DDS message
            data_bytes = bytes(msg.data) if isinstance(msg.data, list) else msg.data

            # ② Convert to numpy array
            img_np = np.frombuffer(data_bytes, dtype=np.uint8)

            # ③ Decode JPEG with OpenCV → Restore BGR image
            frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)

            # ④ Store in cache if successful
            if frame is not None:
                self.cache["/camera/image/compressed"] = frame
        except Exception as e:
            print(f"[RobotisDDSSDK] ⚠ Compressed image decode error: {e}")


    def _image_callback(self, msg: Image_):
        """Handle /camera/image — numpy conversion"""
        try:
            img_data = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            frame = np.frombuffer(img_data, dtype=np.uint8)

            if msg.encoding == "mono8":
                frame = frame.reshape((msg.height, msg.width))
            elif msg.encoding in ["bgr8", "rgb8"]:
                frame = frame.reshape((msg.height, msg.width, 3))
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                print(f"[RobotisDDSSDK] ⚠ Unsupported encoding: {msg.encoding}")
                return

            self.cache["/camera/image"] = frame
        except Exception as e:
            print(f"[RobotisDDSSDK] ⚠ Image decode error: {e}")

    def _odom_callback(self, msg: Odometry_):
        self.cache["/odom"] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": msg.pose.pose.orientation.z,
            "linear_vel": msg.twist.twist.linear.x,
            "angular_vel": msg.twist.twist.angular.z,
        }

    def _joint_state_callback(self, msg: JointState_):
        self.cache["/joint_states"] = dict(zip(msg.name, msg.position))

    def _battery_callback(self, msg: BatteryState_):
        self.cache["/battery_state"] = {
            "voltage": msg.voltage,
            "percentage": msg.percentage,
        }

    # ----------------------------------------------------------------------
    # Public GET (Read sensor data)
    # ----------------------------------------------------------------------
    def get(self, topic_name: str):
        """Generic getter — auto-subscribes if not yet subscribed."""
        self._ensure_subscription(topic_name)
        return self.cache.get(topic_name)

    def get_image(self):
        """Return latest Image (numpy converted /camera/image)"""
        return self.get("/camera/image")

    def get_rgb_image(self):
        """Return latest compressed image (/camera/image/compressed)"""
        return self.get("/camera/image/compressed")

    def get_odometry(self):
        return self.get("/odom")

    def get_joint_state(self):
        return self.get("/joint_states")

    def get_battery_state(self):
        return self.get("/battery_state")

    # ----------------------------------------------------------------------
    # Public SEND (Publish messages)
    # ----------------------------------------------------------------------
    def send_cmd_vel(self, linear_x: float, angular_z: float):
        """Publish a Twist message to /cmd_vel."""
        msg = Twist_(
            linear=Vector3_(x=linear_x, y=0.0, z=0.0),
            angular=Vector3_(x=0.0, y=0.0, z=angular_z),
        )
        self.cmd_vel_pub.publish(msg)

    def send_joint_trajectory(self, positions: list[float]):
        """Publish a JointTrajectory message (SDK handles message format)."""
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)

        header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="base_link")

        point = JointTrajectoryPoint_(
            positions=positions,
            velocities=[],
            accelerations=[],
            effort=[],
            time_from_start=Time_(sec=1, nanosec=0),
        )

        msg = JointTrajectory_(
            header=header,
            joint_names=[f"joint_{i+1}" for i in range(len(positions))],
            points=[point],
        )

        self.joint_traj_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Cleanup
    # ----------------------------------------------------------------------
    def close(self):
        """Shutdown DDS node and stop threads."""
        self.node.dds_destroy_node()
        print("[RobotisDDSSDK] 🧩 DDS closed.")
