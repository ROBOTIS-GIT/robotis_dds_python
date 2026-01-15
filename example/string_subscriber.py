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

from cyclonedds.core import Qos, Policy
from cyclonedds.util import duration
from robotis_dds_python.tools.topic_manager import TopicManager

from robotis_dds_python.idl.std_msgs.msg import String_

qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)

topic_manager = TopicManager()

reader = topic_manager.topic_reader(
    topic_name="/test/string",
    topic_type=String_,
    qos=qos
)

print("[Subscriber] Waiting for messages...")

try:
    while True:
        for sample in reader.take_iter():
            print("[Subscriber] Received:", sample.data)
except KeyboardInterrupt:
    print("Stop")
