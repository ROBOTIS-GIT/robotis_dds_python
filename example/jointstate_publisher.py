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

from robotis_dds_python.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.tools.topic_manager import TopicManager


qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)

# Use the utility function to create the writer
topic_manager = TopicManager()
writer = topic_manager.topic_writer(topic_name='/joint_states', topic_type=JointState_ )


def create_joint_state(t: float) -> JointState_:
    joint_names = ["j1", "j2", "j3"]
    positions = []
    velocities = []
    efforts = []

    for i in range(3):
        positions.append(math.sin(t + i))
        velocities.append(math.cos(t + i))
        efforts.append(0.0)

    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)

    header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="base_link")

    return JointState_(
        header=header,
        name=joint_names,
        position=positions,
        velocity=velocities,
        effort=efforts
    )


t = 0.0

try:
    while True:
        t += 0.1
        msg = create_joint_state(t)
        writer.write(msg)
        print(f"Published {len(msg.name)} joints")
        time.sleep(1.0)
except KeyboardInterrupt:
    print("\nPublisher stopped.")
