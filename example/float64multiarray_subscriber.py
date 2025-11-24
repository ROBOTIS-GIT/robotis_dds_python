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
# Author: Heewon Lee

import time
from cyclonedds.core import Qos, Policy
from cyclonedds.util import duration

from robotis_dds_python.tools.topic_manager import TopicManager

from robotis_dds_python.idl.std_msgs.msg import Float64MultiArray_
from robotis_dds_python.idl.std_msgs.msg import MultiArrayLayout_
from robotis_dds_python.idl.std_msgs.msg import MultiArrayDimension_


qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(10)
)

topic_manager = TopicManager()

reader = topic_manager.topic_reader(
    topic_name="/test/float64_multiarray",
    topic_type=Float64MultiArray_,
    qos=qos
)

print("=== Float64MultiArray Subscriber Started ===")

try:
    while True:
        try:
            for sample in reader.take_iter():
                msg: Float64MultiArray_ = sample

                print("\n[Subscriber] Received Float64MultiArray:")
                print("  data:", list(msg.data))

                layout: MultiArrayLayout_ = msg.layout
                print("  layout.data_offset:", layout.data_offset)

                print("  layout.dim:")
                for d in layout.dim:
                    print(f"    - label={d.label}, size={d.size}, stride={d.stride}")

        except Exception as e:
            print("[Error reading DDS]:", e)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nSubscriber stopped.")