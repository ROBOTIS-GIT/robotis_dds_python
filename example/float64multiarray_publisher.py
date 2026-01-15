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

from robotis_dds_python.idl.std_msgs.msg import MultiArrayDimension_
from robotis_dds_python.idl.std_msgs.msg import MultiArrayLayout_
from robotis_dds_python.idl.std_msgs.msg import Float64MultiArray_


qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)

topic_manager = TopicManager()

writer = topic_manager.topic_writer(
    topic_name="/test/float64_multiarray",
    topic_type=Float64MultiArray_,
    qos=qos
)


def create_multiarray(i: int) -> Float64MultiArray_:
    dim_row = MultiArrayDimension_(
        label="rows",
        size=2,
        stride=2
    )
    dim_col = MultiArrayDimension_(
        label="cols",
        size=2,
        stride=1
    )

    layout = MultiArrayLayout_(
        dim=[dim_row, dim_col],
        data_offset=0
    )

    data = [i * 1.0, i + 1.0, i + 2.0, i + 3.0]

    msg = Float64MultiArray_(
        layout=layout,
        data=data
    )

    return msg


try:
    i = 0
    while True:
        message = create_multiarray(i)
        writer.write(message)
        print(f"[Publisher] Published data: {list(message.data)}")
        i += 1
        time.sleep(1.0)

except KeyboardInterrupt:
    print("\nPublisher stopped.")
