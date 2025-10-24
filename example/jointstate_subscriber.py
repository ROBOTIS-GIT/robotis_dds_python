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

from cyclonedds.core import Listener, Qos, Policy
from cyclonedds.util import duration

from robotis_dds_python.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.tools.topic_manager import TopicManager


class MyListener(Listener):
    def on_liveliness_changed(self, reader, status):
        print(">> Liveliness event")


listener = MyListener()
qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(10)
)

topic_manager = TopicManager()
reader = topic_manager.topic_reader(topic_name="/joint_states", topic_type=JointState_, qos=qos, listener=listener)

try:
    while True:
        try:
            for sample in reader.take_iter():
                if sample:
                    print(f"[JointState Sub] {sample.name} → pos={sample.position}, vel={sample.velocity}, eff={sample.effort}")
        except Exception as e:
            print(f"Error: {e}")
except KeyboardInterrupt:
    print("\nSubscriber stopped.")
