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
import json
import numpy as np
import cv2

from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode
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
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import (
    InferenceAction_,
)


class RobotisDDSSDK:
    """
    Robotis DDS Python SDK
    - Supports camera key mapping via config.json
    - Provides unified access to sensors, cameras, publishers, and services
    """

    def __init__(self, domain_id=30, camera_config_path="config.json"):
        # DDS Node initialization
        self.node = DDSNode(
            name="robotis_sdk_node",
            domain_id=domain_id,
            network_interface="auto",
            allow_multicast=True,
        )

        # Sensor/frame cache
        self.cache = {}

        # Keeps track of already-subscribed default topics
        self._subscribed = {}

        # Camera key → topic mapping (loaded from config.json)
        # Example: { "cam_head": "/zed/left/image_raw/compressed" }
        self._camera_key_map = {}

        # Default non-camera topics
        self.topic_map = {
            "/camera/image": (Image_, self._image_callback),
            "/camera/image/compressed": (CompressedImage_, self._compressed_image_callback),
            "/odom": (Odometry_, self._odom_callback),
            "/joint_states": (JointState_, self._joint_state_callback),
            "/battery_state": (BatteryState_, self._battery_callback),
        }

        # Load camera mappings from config.json and subscribe automatically
        self._load_camera_config(camera_config_path)

        # Publishers
        self.cmd_vel_pub = self.node.dds_create_publisher("/cmd_vel", Twist_)
        self.joint_traj_pub = self.node.dds_create_publisher("/joint_trajectory", JointTrajectory_)
        self.inference_action_pub = self.node.dds_create_publisher("/inference/action", InferenceAction_)

        # Services
        self.ping_client = self.node.dds_create_client(
            "/inference/ping", Ping_Request, Ping_Response
        )
        self.kill_client = self.node.dds_create_client(
            "/inference/kill", Kill_Request, Kill_Response
        )

        # Spin thread
        self.spin_thread = threading.Thread(target=self.node.dds_spin, daemon=True)
        self.spin_thread.start()

    # ---------------------------------------------------------
    # Load camera topics from config.json
    # ---------------------------------------------------------
    def _load_camera_config(self, config_path: str):
        """
        Loads camera_topics from config.json.
        Automatically registers subscriptions for each camera.

        Expected structure:
        {
            "camera_topics": {
                "cam_head": { "topic": "...", "type": "CompressedImage_" },
                "cam_left": { "topic": "...", "type": "Image_" }
            }
        }
        """
        try:
            with open(config_path, "r") as f:
                cfg = json.load(f)
        except Exception as e:
            print(f"[RobotisDDSSDK] Failed to load camera config ({config_path}): {e}")
            return

        cam_cfg = cfg.get("camera_topics", {})
        if not cam_cfg:
            return

        # Map type strings to actual message classes
        type_map = {
            "CompressedImage_": CompressedImage_,
            "Image_": Image_,
        }

        # Register each camera key
        for key, info in cam_cfg.items():
            topic = info.get("topic")
            type_name = info.get("type", "CompressedImage_")
            if not topic:
                continue
            if type_name not in type_map:
                print(f"[RobotisDDSSDK] Unknown camera type '{type_name}' for '{key}'")
                continue

            msg_type = type_map[type_name]
            self._camera_key_map[key] = topic

            print(f"[RobotisDDSSDK] Camera loaded: key='{key}', topic='{topic}', type='{type_name}'")

            # Subscribe to the camera topic
            self.node.dds_create_subscription(
                topic,
                msg_type,
                lambda msg, k=key, t=type_name, tp=topic: self._camera_callback(k, t, tp, msg)
            )

    # ---------------------------------------------------------
    # Lazy subscription for default topics
    # ---------------------------------------------------------
    def _ensure_subscription(self, topic):
        if topic in self._subscribed:
            return
        if topic not in self.topic_map:
            return

        msg_type, cb = self.topic_map[topic]
        self.node.dds_create_subscription(topic, msg_type, cb)
        self._subscribed[topic] = True

    # ---------------------------------------------------------
    # Image Processing
    # ---------------------------------------------------------
    def _decode_compressed(self, msg):
        """Decode CompressedImage_ into an ndarray."""
        try:
            data_bytes = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            img_np = np.frombuffer(data_bytes, dtype=np.uint8)
            return cv2.imdecode(img_np, cv2.IMREAD_COLOR)
        except Exception:
            return None

    def _compressed_image_callback(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/camera/image/compressed"] = frame

    def _camera_callback(self, key, type_name, topic, msg):
        """
        Unified camera callback for all user-defined cameras.
        Automatically decodes compressed or raw formats.
        """
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

        except Exception as e:
            print(f"[RobotisDDSSDK] Camera callback error (key={key}, topic={topic}): {e}")

    # Raw (non-config) image handler
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
        except:
            pass

    # ---------------------------------------------------------
    # Odometry, JointState, Battery
    # ---------------------------------------------------------
    def _odom_callback(self, msg: Odometry_):
        self.cache["/odom"] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": msg.pose.pose.orientation.z,
            "linear_vel": msg.twist.twist.linear.x,
            "angular_vel": msg.twist.twist.angular.z,
        }

    def _joint_state_callback(self, msg: JointState_):
        try:
            self.cache["/joint_states"] = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }
        except Exception:
            self.cache["/joint_states"] = None

    def _battery_callback(self, msg: BatteryState_):
        self.cache["/battery_state"] = {
            "voltage": msg.voltage,
            "percentage": msg.percentage,
        }

    # ---------------------------------------------------------
    # Getters
    # ---------------------------------------------------------
    def get(self, topic):
        """Fetch cached value of a default DDS topic (lazy subscribe)."""
        self._ensure_subscription(topic)
        return self.cache.get(topic)

    def get_image(self): return self.get("/camera/image")
    def get_rgb_image(self): return self.get("/camera/image/compressed")
    def get_odometry(self): return self.get("/odom")
    def get_joint_state(self): return self.get("/joint_states")
    def get_battery_state(self): return self.get("/battery_state")

    def get_images(self, keys=None):
        """
        Get camera frames using camera keys.

        Example:
            sdk.get_images(["cam_head", "cam_left"])

        Returns:
            { "cam_head": ndarray, "cam_left": ndarray }
        """
        if keys is None:
            return self.cache

        out = {}
        for key in keys:
            topic = self._camera_key_map.get(key)
            if topic:
                frame = self.cache.get(topic)
                if frame is not None:
                    out[key] = frame
        return out

    # ---------------------------------------------------------
    # Publishers
    # ---------------------------------------------------------
    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist_(
            linear=Vector3_(x=linear_x, y=0.0, z=0.0),
            angular=Vector3_(x=0.0, y=0.0, z=angular_z),
        )
        self.cmd_vel_pub.publish(msg)

    def send_joint_trajectory(self, positions):
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)

        header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="base_link")
        point = JointTrajectoryPoint_(
            positions=positions,
            velocities=[], accelerations=[], effort=[],
            time_from_start=Time_(sec=1, nanosec=0),
        )
        msg = JointTrajectory_(
            header=header,
            joint_names=[f"joint_{i+1}" for i in range(len(positions))],
            points=[point],
        )
        self.joint_traj_pub.publish(msg)

    def publish_inference_action(self, **fields):
        msg = InferenceAction_(**fields)
        self.inference_action_pub.publish(msg)

    # ---------------------------------------------------------
    # Services
    # ---------------------------------------------------------
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
