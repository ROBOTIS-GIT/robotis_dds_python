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
# Author: Taehyeong Kim


import os
from typing import Type

from cyclonedds.core import Listener, Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.util import duration


class TopicManager:
    def __init__(self, domain_id: int = None):
        if domain_id is None:
            try:
                domain_id = int(os.getenv("ROS_DOMAIN_ID", 0))
            except (ValueError, TypeError):
                domain_id = 0
        self.domain_participant = DomainParticipant(domain_id)

    def topic_reader(self, topic_name: str = "", topic_type: Type = None, qos: Qos = None, listener: Listener = None):
        """
        Create a DDS DataReader with the specified topic name and domain ID.

        :param topic_name: The topic name for the DataReader.
        :param qos: Optional QoS settings for the DataReader.
        :return: A configured DataReader instance.
        """
        if qos is None:
            qos = Qos(
                Policy.Reliability.Reliable(duration()),
                Policy.Durability.Volatile,
                Policy.History.KeepLast(10)
            )
        if listener is None:
            listener = Listener()

        if topic_name.startswith('rt/'):
            final_topic_name = topic_name
        elif topic_name.startswith('/'):
            final_topic_name = 'rt' + topic_name
        else:
            final_topic_name = 'rt/' + topic_name

        topic = Topic(self.domain_participant, final_topic_name, topic_type, qos=qos)
        reader = DataReader(self.domain_participant, topic, listener=listener)
        return reader

    def topic_writer(self, topic_name: str = "", topic_type: Type = None, qos: Qos = None):
        """
        Create a DDS Publisher with the specified topic name and domain ID.

        :param topic_name: The topic name for the DataWriter.
        :param qos: Optional QoS settings for the DataWriter.
        :return: A configured Publisher instance.
        """
        if qos is None:
            qos = Qos(
                Policy.Reliability.Reliable(duration()),
                Policy.Durability.Volatile,
                Policy.History.KeepLast(10)
            )

        if topic_name.startswith('rt/'):
            final_topic_name = topic_name
        elif topic_name.startswith('/'):
            final_topic_name = 'rt' + topic_name
        else:
            final_topic_name = 'rt/' + topic_name

        topic = Topic(self.domain_participant, final_topic_name, topic_type, qos=qos)
        publisher = Publisher(self.domain_participant, qos=qos)
        writer = DataWriter(publisher, topic)
        return writer
