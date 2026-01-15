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

from robotis_dds_python.idl.std_msgs.msg import String_

qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)

topic_manager = TopicManager()

writer = topic_manager.topic_writer(
    topic_name="/test/string",
    topic_type=String_,
    qos=qos
)

i = 0
try:
    while True:
        msg = String_(data=f"Hello DDS {i}")
        writer.write(msg)
        print("[Publisher] Sent:", msg.data)
        i += 1
        time.sleep(1)
except KeyboardInterrupt:
    print("Stop")
