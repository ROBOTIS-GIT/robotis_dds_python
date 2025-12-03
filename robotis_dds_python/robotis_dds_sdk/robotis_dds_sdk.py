#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Author: Heewon Lee

import threading
import time
import numpy as np
import cv2
import json

from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_, Duration_
from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import (
    CompressedImage_, Image_, JointState_, BatteryState_
)
from robotis_dds_python.robotis_dds_core.idl.nav_msgs.msg import Odometry_
from robotis_dds_python.robotis_dds_core.idl.geometry_msgs.msg import Twist_, Vector3_
from robotis_dds_python.robotis_dds_core.idl.trajectory_msgs.msg import (
    JointTrajectory_, JointTrajectoryPoint_
)
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.srv import (
    Ping_Request, Ping_Response,
    Kill_Request, Kill_Response,
)
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import InferenceAction_


class RobotisDDSSDK:
    """High-level DDS wrapper for Robot sensors & actuators."""

    def __init__(self, domain_id=30, robot_type=None):
        # --- Create DDS node ---
        self.node = DDSNode(
            name="robotis_sdk_node",
            domain_id=domain_id,
            network_interface="auto",
            allow_multicast=True,
        )

        # --- Cache for last sensor values ---
        self.cache = {}
        self._subscribed = {}

        # --- Mappings for cameras & arm publishers ---
        self._camera_key_map = {}   # key → topic
        self._arm_pubs = {}         # arm → publisher
        self._arm_traj_epoch_ns = {}  # arm → time_from_start epoch
        self._arm_last_tfs_ns = {}     # arm → last TFS ns

        # --- Default topics ---
        self._odom_topic = "/odom"
        self._joint_states_topic = "/joint_states"
        self._battery_topic = "/battery_state"

        # --- Lazy subscription mapping ---
        self.topic_map = {
            "/camera/image": (Image_, self._image_callback),
            "/camera/image/compressed": (CompressedImage_, self._compressed_image_callback),
            self._odom_topic: (Odometry_, self._odom_callback),
            self._joint_states_topic: (JointState_, self._joint_state_callback),
            self._battery_topic: (BatteryState_, self._battery_callback),
        }

        # --- Publishers ---
        self.cmd_vel_pub = self.node.dds_create_publisher("/cmd_vel", Twist_)
        self.joint_traj_pub = self.node.dds_create_publisher("/joint_trajectory", JointTrajectory_)
        self.inference_action_pub = self.node.dds_create_publisher("/inference/action", InferenceAction_)

        # --- Service Clients ---
        self.ping_client = self.node.dds_create_client("/inference/ping", Ping_Request, Ping_Response)
        self.kill_client = self.node.dds_create_client("/inference/kill", Kill_Request, Kill_Response)

        # Robot type for config.json auto-registration
        self.robot_type = robot_type

        # --- Load configuration & register sensors/publishers ---
        self._load_config_and_register()

        # --- DDS spin thread ---
        self.spin_thread = threading.Thread(target=self.node.dds_spin, daemon=True)
        self.spin_thread.start()

    # ------------------------------------------------------------
    # Config loader: Reads config.json and registers cameras/arms
    # ------------------------------------------------------------
    def _load_config_and_register(self, cfg_path="config.json"):
        """
        Load config.json (auto camera & publisher registration).
        WARNING:
            * Modify config.json instead of touching SDK code.
        """
        try:
            with open(cfg_path, "r") as f:
                cfg = json.load(f)
        except:
            return

        if not self.robot_type or self.robot_type not in cfg:
            return

        rob_cfg = cfg[self.robot_type]

        # --- Register cameras from config.json ---
        for key, v in rob_cfg.get("camera_topics", {}).items():
            topic = v.get("topic")
            msg_type = v.get("type", "CompressedImage_")
            if topic:
                self.register_camera(key, topic, msg_type)

        # --- Register arm publishers from config.json ---
        for arm, topic in rob_cfg.get("arm_publishers", {}).items():
            self.register_arm_publisher(arm, topic)

    # ------------------------------------------------------------
    # Subscription helpers
    # ------------------------------------------------------------
    def _ensure_subscription(self, topic):
        """Create subscription on first access."""
        if topic in self._subscribed or topic not in self.topic_map:
            return
        msg_type, cb = self.topic_map[topic]
        self.node.dds_create_subscription(topic, msg_type, cb)
        self._subscribed[topic] = True

    # ------------------------------------------------------------
    # Image decoding utilities
    # ------------------------------------------------------------
    def _decode_compressed(self, msg):
        """Decode sensor_msgs/CompressedImage into OpenCV BGR frame."""
        try:
            data_bytes = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            arr = np.frombuffer(data_bytes, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except:
            return None

    # DDS callback for /camera/image/compressed
    def _compressed_image_callback(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/camera/image/compressed"] = frame

    # Generic callback for custom cameras registered via config.json
    def _camera_callback(self, key, type_name, topic, msg):
        """Unified callback handler for all cameras registered from config.json."""
        try:
            if type_name == "CompressedImage_":
                frame = self._decode_compressed(msg)
            else:
                raw = bytes(msg.data) if isinstance(msg.data, list) else msg.data
                arr = np.frombuffer(raw, dtype=np.uint8)
                if msg.encoding == "mono8":
                    frame = arr.reshape((msg.height, msg.width))
                else:
                    frame = arr.reshape((msg.height, msg.width, 3))
                    if msg.encoding == "rgb8":
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if frame is not None:
                self.cache[topic] = frame
        except:
            pass

    # DDS callback for raw /camera/image
    def _image_callback(self, msg):
        """Decode Image_ messages."""
        try:
            raw = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            arr = np.frombuffer(raw, dtype=np.uint8)
            if msg.encoding == "mono8":
                frame = arr.reshape((msg.height, msg.width))
            else:
                frame = arr.reshape((msg.height, msg.width, 3))
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.cache["/camera/image"] = frame
        except:
            pass

    # ------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------
    def _odom_callback(self, msg: Odometry_):
        """Cache odometry values."""
        self.cache[self._odom_topic] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": msg.pose.pose.orientation.z,
            "linear_vel": msg.twist.twist.linear.x,
            "angular_vel": msg.twist.twist.angular.z,
        }

    def _joint_state_callback(self, msg: JointState_):
        """Cache full joint state."""
        try:
            self.cache[self._joint_states_topic] = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }
        except:
            self.cache[self._joint_states_topic] = None

    def _battery_callback(self, msg: BatteryState_):
        """Cache battery percentage & voltage."""
        self.cache[self._battery_topic] = {
            "voltage": msg.voltage,
            "percentage": msg.percentage,
        }

    # ------------------------------------------------------------
    # Public getters (auto lazy-subscribe)
    # ------------------------------------------------------------
    def get(self, topic):
        """Retrieve cached data (ensures subscription)."""
        self._ensure_subscription(topic)
        return self.cache.get(topic)

    def get_image(self):
        return self.get("/camera/image")

    def get_rgb_image(self):
        return self.get("/camera/image/compressed")

    def get_odometry(self):
        return self.get(self._odom_topic)

    def get_joint_state(self):
        return self.get(self._joint_states_topic)

    def get_battery_state(self):
        return self.get(self._battery_topic)

    def get_camera(self, key: str):
        """Get camera frame by config.json name."""
        topic = self._camera_key_map.get(key)
        return self.cache.get(topic) if topic else None

    def get_images(self):
        """Return all registered camera frames as dict(key → frame)."""
        out = {}
        for key, topic in self._camera_key_map.items():
            frame = self.cache.get(topic)
            if frame is not None:
                out[key] = frame
        return out

    # ------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------
    def send_cmd_vel(self, linear_x, angular_z):
        """Publish Twist command on /cmd_vel."""
        msg = Twist_(
            linear=Vector3_(x=linear_x, y=0.0, z=0.0),
            angular=Vector3_(x=0.0, y=0.0, z=angular_z),
        )
        self.cmd_vel_pub.publish(msg)

    def _make_timestamp(self):
        """Generate ROS2-style timestamp from system wall clock."""
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        return Time_(sec=sec, nanosec=nsec)

    def send_joint_trajectory(self, positions):
        """Publish a single joint trajectory command."""
        stamp = self._make_timestamp()
        header = Header_(stamp=stamp, frame_id="")

        point = JointTrajectoryPoint_(
            positions=positions,
            velocities=[],
            accelerations=[],
            effort=[],
            time_from_start=Time_(sec=0, nanosec=0),
        )

        msg = JointTrajectory_(
            header=header,
            joint_names=[f"joint_{i+1}" for i in range(len(positions))],
            points=[point],
        )

        self.joint_traj_pub.publish(msg)

    # ------------------------------------------------------------
    # Camera / Arm registration (config.json-driven)
    # ------------------------------------------------------------
    def register_camera(self, key, topic, type_name="CompressedImage_"):
        """
        Register a camera from config.json.
        DO NOT modify code; edit config.json under camera_topics.
        """
        self._camera_key_map[key] = topic
        msg_type = CompressedImage_ if type_name == "CompressedImage_" else Image_

        def cb(msg, k=key, t=topic, ty=type_name):
            self._camera_callback(k, ty, t, msg)

        self.node.dds_create_subscription(topic, msg_type, cb)

    def register_arm_publisher(self, arm: str, topic: str):
        """Create publisher for arm trajectory (configured via config.json)."""
        pub = self.node.dds_create_publisher(topic, JointTrajectory_)
        self._arm_pubs[arm] = pub

        # Initialize time_from_start tracking
        now_ns = time.monotonic_ns()
        self._arm_traj_epoch_ns[arm] = now_ns
        self._arm_last_tfs_ns[arm] = 0

    def reset_arm_tfs(self, arm: str):
        """Reset time_from_start counter (required after controller restart)."""
        self._arm_traj_epoch_ns[arm] = time.monotonic_ns()
        self._arm_last_tfs_ns[arm] = 0

    # ------------------------------------------------------------
    # Trajectory publishing with strict time_from_start increments
    # ------------------------------------------------------------
    def send_arm_trajectory(self, arm: str, positions, dt: float = 0.03, velocities=None, fast_mode: bool = True):
        """
        Publish JointTrajectory ensuring:
        * time_from_start strictly increases (required by ros2_control)
        * Optional velocities per point
        """
        if arm not in self._arm_pubs:
            return

        # --- Header stamp ---
        t_wall = time.time()
        sec = int(t_wall)
        nsec = int((t_wall - sec) * 1e9)
        header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="")

        # --- Normalize input ---
        is_multi = isinstance(positions[0], (list, tuple))
        pos_list = positions if is_multi else [positions]

        vel_list = None
        if velocities is not None:
            vel_list = velocities if isinstance(velocities[0], (list, tuple)) else [velocities]
            if len(vel_list) != len(pos_list):
                vel_list = None

        # --- Controller timing ---
        dt = max(0.01, float(dt))
        base_delay = max(dt, 0.06) if fast_mode else max(dt, 0.12)

        last_tfs_ns = self._arm_last_tfs_ns.get(arm, 0)

        # Ensure strictly increasing monotonic tfs
        batch_gap_ns = int((0.02 if fast_mode else 0.05) * 1e9)
        start_ns = max(last_tfs_ns + batch_gap_ns, int(base_delay * 1e9))

        points = []
        for i, pos in enumerate(pos_list):
            tfs_ns = start_ns + int(i * dt * 1e9)
            vels = vel_list[i] if vel_list else []
            points.append(
                JointTrajectoryPoint_(
                    positions=list(pos),
                    velocities=list(vels),
                    accelerations=[],
                    effort=[],
                    time_from_start=Duration_(sec=tfs_ns // 1_000_000_000,
                                              nanosec=tfs_ns % 1_000_000_000),
                )
            )

        # --- Final publish ---
        try:
            JOINT_NAME_MAP = {
                "left": [
                    'arm_l_joint1','arm_l_joint2','arm_l_joint3','arm_l_joint4',
                    'arm_l_joint5','arm_l_joint6','arm_l_joint7','gripper_l_joint1'
                ],
                "right": [
                    'arm_r_joint1','arm_r_joint2','arm_r_joint3','arm_r_joint4',
                    'arm_r_joint5','arm_r_joint6','arm_r_joint7','gripper_r_joint1'
                ]
            }

            msg = JointTrajectory_(
                header=header,
                joint_names=JOINT_NAME_MAP.get(arm, []),
                points=points,
            )

            self._arm_pubs[arm].publish(msg)

            # Update last tfs
            last_point = points[-1].time_from_start
            self._arm_last_tfs_ns[arm] = last_point.sec * 1_000_000_000 + last_point.nanosec

        except Exception as e:
            print(f"[ERROR] send_arm_trajectory({arm}) failed: {e}")

    # ------------------------------------------------------------
    # Services
    # ------------------------------------------------------------
    def ping(self):
        """Call /inference/ping service."""
        try:
            return self.ping_client.call(Ping_Request())
        except Exception as e:
            return Ping_Response(success=False, message=str(e))

    def kill(self):
        """Call /inference/kill service."""
        try:
            return self.kill_client.call(Kill_Request())
        except Exception as e:
            return Kill_Response(success=False, message=str(e))
