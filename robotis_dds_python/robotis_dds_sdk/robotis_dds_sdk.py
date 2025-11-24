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
    """Robotis DDS Python SDK (clean version, no debug prints)."""

    def __init__(self, domain_id=30):
        self.node = DDSNode(
            name="robotis_sdk_node",
            domain_id=domain_id,
            network_interface="auto",
            allow_multicast=True,
        )

        self.cache = {}
        self._subscribed = {}

        # Topic map
        self.topic_map = {
            "/camera/image": (Image_, self._image_callback),
            "/camera/image/compressed": (CompressedImage_, self._compressed_image_callback),
            "/odom": (Odometry_, self._odom_callback),
            "/joint_states": (JointState_, self._joint_state_callback),
            "/battery_state": (BatteryState_, self._battery_callback),

            "/camera_left/camera_left/color/image_rect_raw/compressed":
                (CompressedImage_, self._compressed_image_callback_left),
            "/camera_right/camera_right/color/image_rect_raw/compressed":
                (CompressedImage_, self._compressed_image_callback_right),
            "/zed/zed_node/left/image_rect_color/compressed":
                (CompressedImage_, self._compressed_image_callback_zed_left),
            "/zed/zed_node/right/image_rect_color/compressed":
                (CompressedImage_, self._compressed_image_callback_zed_right),
        }

        # Publishers
        self.cmd_vel_pub = self.node.dds_create_publisher("/cmd_vel", Twist_)
        self.joint_traj_pub = self.node.dds_create_publisher("/joint_trajectory", JointTrajectory_)
        self.inference_action_pub = self.node.dds_create_publisher("/inference/action", InferenceAction_)

        # Service Clients
        self.ping_client = self.node.dds_create_client(
            "/inference/ping", Ping_Request, Ping_Response
        )
        self.kill_client = self.node.dds_create_client(
            "/inference/kill", Kill_Request, Kill_Response
        )

        # Start spin thread
        self.spin_thread = threading.Thread(target=self.node.dds_spin, daemon=True)
        self.spin_thread.start()

    # ------------------------------
    # Subscription Manager
    # ------------------------------
    def _ensure_subscription(self, topic):
        if topic in self._subscribed:
            return
        if topic not in self.topic_map:
            return
        msg_type, cb = self.topic_map[topic]
        self.node.dds_create_subscription(topic, msg_type, cb)
        self._subscribed[topic] = True

    # ------------------------------
    # Image Handlers
    # ------------------------------
    def _decode_compressed(self, msg):
        try:
            data_bytes = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            img_np = np.frombuffer(data_bytes, dtype=np.uint8)
            return cv2.imdecode(img_np, cv2.IMREAD_COLOR)
        except:
            return None

    def _compressed_image_callback(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/camera/image/compressed"] = frame

    def _compressed_image_callback_left(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/camera_left/camera_left/color/image_rect_raw/compressed"] = frame

    def _compressed_image_callback_right(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/camera_right/camera_right/color/image_rect_raw/compressed"] = frame

    def _compressed_image_callback_zed_left(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/zed/zed_node/left/image_rect_color/compressed"] = frame

    def _compressed_image_callback_zed_right(self, msg):
        frame = self._decode_compressed(msg)
        if frame is not None:
            self.cache["/zed/zed_node/right/image_rect_color/compressed"] = frame

    def get_images(self):
        return self.cache
    # ------------------------------
    # Raw Image
    # ------------------------------
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

    # ------------------------------
    # Odom
    # ------------------------------
    def _odom_callback(self, msg: Odometry_):
        self.cache["/odom"] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": msg.pose.pose.orientation.z,
            "linear_vel": msg.twist.twist.linear.x,
            "angular_vel": msg.twist.twist.angular.z,
        }

    # ------------------------------
    # Joint State
    # ------------------------------
    def _joint_state_callback(self, msg: JointState_):
        try:
            self.cache["/joint_states"] = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }
        except:
            self.cache["/joint_states"] = None

    # ------------------------------
    # Battery
    # ------------------------------
    def _battery_callback(self, msg: BatteryState_):
        self.cache["/battery_state"] = {
            "voltage": msg.voltage,
            "percentage": msg.percentage,
        }

    # ------------------------------
    # Getters
    # ------------------------------
    def get(self, topic):
        self._ensure_subscription(topic)
        return self.cache.get(topic)

    def get_image(self): return self.get("/camera/image")
    def get_rgb_image(self): return self.get("/camera/image/compressed")
    def get_left_image(self): return self.get("/camera_left/camera_left/color/image_rect_raw/compressed")
    def get_right_image(self): return self.get("/camera_right/camera_right/color/image_rect_raw/compressed")
    def get_zed_left_image(self): return self.get("/zed/zed_node/left/image_rect_color/compressed")
    def get_zed_right_image(self): return self.get("/zed/zed_node/right/image_rect_color/compressed")
    def get_odometry(self): return self.get("/odom")
    def get_joint_state(self): return self.get("/joint_states")
    def get_battery_state(self): return self.get("/battery_state")

    # ------------------------------
    # Publishing
    # ------------------------------
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

    # ------------------------------
    # Services
    # ------------------------------
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
