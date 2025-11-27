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
    - config.json의 robot_type 블록 기준으로 camera / arm / sensor 자동 등록
    - Unified access to sensors, cameras, publishers, and services
    """

    def __init__(self, domain_id=30, robot_type=None):
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

        # Camera key → topic mapping in config.json
        # Example: { "cam_head": "/zed/left/image_raw/compressed" }
        self._camera_key_map = {}

        # Arm publishers (left/right 등)
        self._arm_pubs = {}

        # 기본 센서 토픽 (config로 override 가능)
        self._odom_topic = "/odom"
        self._joint_states_topic = "/joint_states"
        self._battery_topic = "/battery_state"

        # Default non-camera topics (키는 실제 DDS 토픽 이름)
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
        self.ping_client = self.node.dds_create_client(
            "/inference/ping", Ping_Request, Ping_Response
        )
        self.kill_client = self.node.dds_create_client(
            "/inference/kill", Kill_Request, Kill_Response
        )

        # ========= ⭐ config.json 로드 + 자동 등록 ⭐ =========
        self.robot_type = robot_type
        self._load_config_and_register()
        print(f"[RobotisDDSSDK] Camera map after config: {self._camera_key_map}")

        # Spin thread
        self.spin_thread = threading.Thread(target=self.node.dds_spin, daemon=True)
        self.spin_thread.start()

    # ---------------------------------------------------------
    # config.json 로드 + robot_type별 자동 등록
    # ---------------------------------------------------------
    def _load_config_and_register(self, cfg_path="config.json"):
        try:
            with open(cfg_path, "r") as f:
                cfg = json.load(f)
        except FileNotFoundError:
            print(f"[RobotisDDSSDK] WARNING: {cfg_path} not found → config not applied.")
            return
        except Exception as e:
            print(f"[RobotisDDSSDK] ERROR: Failed to load {cfg_path}: {e}")
            return

        # -----------------------------
        # ⭐ 로봇 타입 선택 (필수)
        # -----------------------------
        if self.robot_type is None:
            print(f"[RobotisDDSSDK] WARNING: robot_type not provided — config not applied.")
            return

        if self.robot_type not in cfg:
            print(f"[RobotisDDSSDK] WARNING: Robot type '{self.robot_type}' not found in config.json")
            return

        rob_cfg = cfg[self.robot_type]

        # ===========================
        # ⭐ other_sensors 처리 (예: joint_states)
        # ===========================
        other_cfg = rob_cfg.get("other_sensors", {})
        joint_topic = other_cfg.get("joint_states")
        if joint_topic:
            self._joint_states_topic = joint_topic
            # topic_map에 해당 토픽 매핑 추가/덮어쓰기
            self.topic_map[joint_topic] = (JointState_, self._joint_state_callback)
            print(f"[RobotisDDSSDK] JointState topic set to: {joint_topic}")

        # (필요하면 odom, battery 등도 여기서 확장 가능)

        # ===========================
        # ⭐ Camera 등록
        # ===========================
        cam_cfg = rob_cfg.get("camera_topics", {})
        for key, v in cam_cfg.items():
            topic = v.get("topic")
            msg_type = v.get("type", "CompressedImage_")
            print(f"[RobotisDDSSDK] Auto-register camera: {key} → {topic} ({msg_type})")
            self.register_camera(key, topic, msg_type)

        # ===========================
        # ⭐ Arm publisher 등록
        # ===========================
        arm_cfg = rob_cfg.get("arm_publishers", {})
        for arm, topic in arm_cfg.items():
            print(f"[RobotisDDSSDK] Auto-register arm publisher: {arm} → {topic}")
            self.register_arm_publisher(arm, topic)

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
        camera_topics에서 등록한 카메라용 콜백
        - key: cam_head / cam_left ...
        - topic: 실제 DDS 토픽명
        - type_name: "CompressedImage_" or "Image_"
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

            # ⭐ 실제 프레임을 cache에 저장해야 get_images()가 쓸 수 있음
            if frame is not None:
                self.cache[topic] = frame

        except Exception as e:
            print(f"[RobotisDDSSDK] Camera callback error (key={key}, topic={topic}): {e}")

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

    # ---------------------------------------------------------
    # Odometry, JointState, Battery
    # ---------------------------------------------------------
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

    # ---------------------------------------------------------
    # Getters
    # ---------------------------------------------------------
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
        # other_sensors.joint_states 값이 있으면 그 토픽을 사용
        return self.get(self._joint_states_topic)

    def get_battery_state(self):
        return self.get(self._battery_topic)

    def get_camera(self, key: str):
        topic = self._camera_key_map.get(key)
        if topic:
            return self.cache.get(topic)
        return None

    def get_images(self):
        """
        Always return all camera frames defined in camera_topics
        from the loaded config.json.
        ex) {"cam_head": np.ndarray, "cam_left": np.ndarray, ...}
        """
        keys = list(self._camera_key_map.keys())

        out = {}
        for key in keys:
            topic = self._camera_key_map.get(key)
            if topic is None:
                continue
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
        """
        기본 /joint_trajectory 토픽으로 전체 팔(또는 1개의 팔)을 보내고 싶을 때 사용.
        (로봇별 left/right 분리는 send_arm_trajectory 사용)
        """
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

    # ---------------------------------------------------------
    # Arm publishers
    # ---------------------------------------------------------
    def register_camera(self, key, topic, type_name="CompressedImage_"):
        """
        Register a camera topic (usually from config.json → camera_topics).
        key: logical name (cam_head, cam_left, ...)
        topic: DDS topic name
        type_name: 'CompressedImage_' or 'Image_'
        """
        self._camera_key_map[key] = topic

        if type_name == "CompressedImage_":
            msg_type = CompressedImage_
        else:
            msg_type = Image_

        # key, type_name, topic을 callback에 전달할 수 있도록 wrapper 생성
        def cb(msg, k=key, t=topic, ty=type_name):
            self._camera_callback(k, ty, t, msg)

        print(f"[RobotisDDSSDK] Registering camera subscription: key={key}, topic={topic}, type={type_name}")
        self.node.dds_create_subscription(topic, msg_type, cb)
        print(f"[RobotisDDSSDK] Camera registered: {key} → {topic}")

    def register_arm_publisher(self, arm: str, topic: str):
        """
        Register an arm trajectory publisher dynamically.
        ex) arm='left', topic='/joint_trajectory_left'
        """
        pub = self.node.dds_create_publisher(topic, JointTrajectory_)
        self._arm_pubs[arm] = pub
        print(f"[RobotisDDSSDK] Arm publisher registered: {arm} → {topic}")

    def send_arm_trajectory(self, arm: str, positions):
        """
        로봇 타입별 팔(왼팔/오른팔 등)에 맞게 config.arm_publishers 기준으로 퍼블리시.
        """
        if arm not in self._arm_pubs:
            print(f"[RobotisDDSSDK] ERROR: No arm publisher registered for '{arm}'")
            return

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
            joint_names=[f"{arm}_joint_{i+1}" for i in range(len(positions))],
            points=[point],
        )

        self._arm_pubs[arm].publish(msg)

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
