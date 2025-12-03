#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Robotis DDS Python SDK + DDS Inference Server
# High-level wrapper for DDS-based robot communication + Inference integration
#
# Author: Heewon Lee, Dongyun Kim
# License: Apache 2.0

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
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.srv import (
    Ping_Request, Ping_Response,
    Kill_Request, Kill_Response,
)
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import InferenceAction_


class RobotisDDSSDK:
    def __init__(self, domain_id=30, robot_type=None):
        self.node = DDSNode(
            name="robotis_sdk_node",
            domain_id=domain_id,
            network_interface="auto",
            allow_multicast=True,
        )

        # Caches
        self.cache = {}
        self._subscribed = {}

        # Mappings
        self._camera_key_map = {}
        self._arm_pubs = {}
        # Per-arm trajectory epoch (monotonic), ensures increasing time_from_start across publishes
        self._arm_traj_epoch_ns = {}  # arm -> int (start time_ns)
        self._arm_last_tfs_ns = {}    # arm -> int (last time_from_start in ns)

        # Default topics
        self._odom_topic = "/odom"
        self._joint_states_topic = "/joint_states"
        self._battery_topic = "/battery_state"

        # Lazy-subscribe topic map
        self.topic_map = {
            "/camera/image": (Image_, self._image_callback),
            "/camera/image/compressed": (CompressedImage_, self._compressed_image_callback),
            self._odom_topic: (Odometry_, self._odom_callback),
            self._joint_states_topic: (JointState_, self._joint_state_callback),
            self._battery_topic: (BatteryState_, self._battery_callback),
        }

        # Publishers
        self.cmd_vel_pub = self.node.dds_create_publisher("/cmd_vel", Twist_)
        self.joint_traj_pub = self.node.dds_create_publisher("/joint_trajectory", JointTrajectory_)
        self.inference_action_pub = self.node.dds_create_publisher("/inference/action", InferenceAction_)

        # Services
        self.ping_client = self.node.dds_create_client("/inference/ping", Ping_Request, Ping_Response)
        self.kill_client = self.node.dds_create_client("/inference/kill", Kill_Request, Kill_Response)

        # Config-based auto registration
        self.robot_type = robot_type
        self._load_config_and_register()

        # Spin
        self.spin_thread = threading.Thread(target=self.node.dds_spin, daemon=True)
        self.spin_thread.start()

    # ---------------- Config loader ----------------
    def _load_config_and_register(self, cfg_path="config.json"):
    
        try:
            with open(cfg_path, "r") as f:
                cfg = json.load(f)
        except Exception:
            return
        if not self.robot_type or self.robot_type not in cfg:
            return
        rob_cfg = cfg[self.robot_type]

        # camera_topics (add new cameras by editing config.json)
        for key, v in rob_cfg.get("camera_topics", {}).items():
            topic = v.get("topic")
            msg_type = v.get("type", "CompressedImage_")
            if topic:
                self.register_camera(key, topic, msg_type)

        # arm_publishers
        for arm, topic in rob_cfg.get("arm_publishers", {}).items():
            self.register_arm_publisher(arm, topic)

    # ---------------- Subscription helpers ----------------
    def _ensure_subscription(self, topic):
        if topic in self._subscribed or topic not in self.topic_map:
            return
        msg_type, cb = self.topic_map[topic]
        self.node.dds_create_subscription(topic, msg_type, cb)
        self._subscribed[topic] = True

    # ---------------- Image decoding ----------------
    def _decode_compressed(self, msg):
        try:
            data_bytes = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            arr = np.frombuffer(data_bytes, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return None

    def _compressed_image_callback(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/camera/image/compressed"] = frame

    def _camera_callback(self, key, type_name, topic, msg):
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
        except Exception:
            pass

    def _image_callback(self, msg):
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
        except Exception:
            pass

    # ---------------- Sensor callbacks ----------------
    def _odom_callback(self, msg: Odometry_):
        self.cache[self._odom_topic] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": msg.pose.pose.orientation.z,
            "linear_vel": msg.twist.twist.linear.x,
            "angular_vel": msg.twist.twist.angular.z,
        }

    def _joint_state_callback(self, msg: JointState_):
        try:
            self.cache[self._joint_states_topic] = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }
        except Exception:
            self.cache[self._joint_states_topic] = None

    def _battery_callback(self, msg: BatteryState_):
        self.cache[self._battery_topic] = {
            "voltage": msg.voltage,
            "percentage": msg.percentage,
        }

    # ---------------- Getters ----------------
    def get(self, topic):
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
        topic = self._camera_key_map.get(key)
        return self.cache.get(topic) if topic else None

    def get_images(self):
        out = {}
        for key, topic in self._camera_key_map.items():
            frame = self.cache.get(topic)
            if frame is not None:
                out[key] = frame
        return out

    # ---------------- Publishers ----------------
    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist_(
            linear=Vector3_(x=linear_x, y=0.0, z=0.0),
            angular=Vector3_(x=0.0, y=0.0, z=angular_z),
        )
        self.cmd_vel_pub.publish(msg)

    def _make_timestamp(self):
        """system time 기반 ROS2 Time stamp 생성 (DDS 환경에서 가장 안전한 방식)"""
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        return time(sec=sec, nanosec=nsec)

    def send_joint_trajectory(self, positions):
        # --- FIXED: 올바른 timestamp 생성 ---
        stamp = self._make_timestamp()
        header = Header_(stamp=stamp, frame_id="")

        # trajectory point
        point = JointTrajectoryPoint_(
            positions=positions,
            velocities=[],
            accelerations=[],
            effort=[],
            time_from_start=Time_(sec=0, nanosec=0),
        )

        # full message
        msg = JointTrajectory_(
            header=header,
            joint_names=[f"joint_{i+1}" for i in range(len(positions))],
            points=[point],
        )

        # publish
        self.joint_traj_pub.publish(msg)

    # ---------------- Camera / Arm registration ----------------
    def register_camera(self, key, topic, type_name="CompressedImage_"):
        """
        Register a camera manually.
        To add permanently: edit config.json camera_topics block.
        """
        self._camera_key_map[key] = topic
        msg_type = CompressedImage_ if type_name == "CompressedImage_" else Image_

        def cb(msg, k=key, t=topic, ty=type_name):
            self._camera_callback(k, ty, t, msg)

        self.node.dds_create_subscription(topic, msg_type, cb)

    def register_arm_publisher(self, arm: str, topic: str):
        pub = self.node.dds_create_publisher(topic, JointTrajectory_)
        self._arm_pubs[arm] = pub
        # initialize monotonic epoch and last tfs
        now_ns = time.monotonic_ns()
        self._arm_traj_epoch_ns[arm] = now_ns
        self._arm_last_tfs_ns[arm] = 0

    def reset_arm_tfs(self, arm: str):
        """
        Reset stored time_from_start for given arm.
        Call this after controller restart or when invalid trajectory error occurred.
        """
        self._arm_traj_epoch_ns[arm] = time.monotonic_ns()
        self._arm_last_tfs_ns[arm] = 0

    def send_arm_trajectory(self, arm: str, positions, dt: float = 0.03, velocities=None, fast_mode: bool = True):
        """
        Publish a trajectory ensuring time_from_start is strictly increasing across calls.
        """
        if arm not in self._arm_pubs:
            return

        # Header stamp
        t_wall = time.time()
        sec = int(t_wall)
        nsec = int((t_wall - sec) * 1e9)
        header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="")

        # Normalize inputs
        is_multi = isinstance(positions[0], (list, tuple))
        pos_list = positions if is_multi else [positions]
        vel_list = None
        if velocities is not None:
            vel_list = velocities if isinstance(velocities[0], (list, tuple)) else [velocities]
            if len(vel_list) != len(pos_list):
                vel_list = None

        # Controller buffer delay
        dt = max(0.01, float(dt))
        base_delay = max(dt, 0.06) if fast_mode else max(dt, 0.12)

        # Compute points with strictly increasing time_from_start
        points = []
        last_tfs_ns = self._arm_last_tfs_ns.get(arm, 0)
        if arm not in self._arm_traj_epoch_ns:
            self._arm_traj_epoch_ns[arm] = time.monotonic_ns()
            last_tfs_ns = 0

        # Gap between batches: keep strictly greater than previous
        batch_gap_ns = int((0.02 if fast_mode else 0.05) * 1e9)
        start_ns = max(last_tfs_ns + batch_gap_ns, int(base_delay * 1e9))

        for i, pos in enumerate(pos_list):
            tfs_ns = start_ns + int(i * dt * 1e9)
            vels = vel_list[i] if vel_list else []
            points.append(
                JointTrajectoryPoint_(
                    positions=list(pos),
                    velocities=list(vels),
                    accelerations=[],
                    effort=[],
                    time_from_start=Duration_(sec=tfs_ns // 1_000_000_000, nanosec=tfs_ns % 1_000_000_000),
                )
            )

        # Build and publish message; update last_tfs_ns only on success
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
            # Update stored last time_from_start (ns)
            last_point = points[-1].time_from_start
            self._arm_last_tfs_ns[arm] = last_point.sec * 1_000_000_000 + last_point.nanosec
        except Exception as e:
            # If publish fails (e.g., node shut down), avoid using dead context
            print(f"[ERROR] send_arm_trajectory({arm}) failed: {e}")

    # ---------------- Services ----------------
    def ping(self):
        try:
            return self.ping_client.call(Ping_Request())
        except Exception as e:
            return Ping_Response(success=False, message=str(e))

    def kill(self):
        try:
            return self.kill_client.call(Kill_Request())
        except Exception as e:
            return Kill_Response(success=False, message=str(e))
