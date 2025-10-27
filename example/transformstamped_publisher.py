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
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Taehyeong Kim, Heewon Lee


import time
import math
from cyclonedds.core import Qos, Policy
from cyclonedds.util import duration

from robotis_dds_python.idl.geometry_msgs.msg import TransformStamped_, Transform_, Vector3_, Quaternion_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.tools.topic_manager import TopicManager


qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)


topic_manager = TopicManager()
writer = topic_manager.topic_writer(
    topic_name="/tf",
    topic_type=TransformStamped_,
    qos=qos
)


def create_transform(t: float) -> TransformStamped_:
    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)

    header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="world")

    translation = Vector3_(x=math.cos(t), y=math.sin(t), z=0.0)
    rotation = Quaternion_(x=0.0, y=0.0, z=math.sin(t / 2), w=math.cos(t / 2))
    transform = Transform_(translation=translation, rotation=rotation)

    return TransformStamped_(header=header, child_frame_id="base_link", transform=transform)


t = 0.0

try:
    while True:
        t += 0.1
        msg = create_transform(t)
        writer.write(msg)
        print(f"Published TransformStamped: "
              f"x={msg.transform.translation.x:.2f}, "
              f"y={msg.transform.translation.y:.2f}, "
              f"frame={msg.header.frame_id}")
        time.sleep(1.0)
except KeyboardInterrupt:
    print("\nPublisher stopped.")