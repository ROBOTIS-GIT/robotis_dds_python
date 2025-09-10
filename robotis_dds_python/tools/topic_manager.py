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
            domain_id = int(os.getenv("ROS_DOMAIN_ID", 0))
        self.domain_participant = DomainParticipant(domain_id)

    def topic_reader(self, topic_name: str = "", topic_type: Type = None, qos: Qos = None):
        """
        Create a DDS DataReader with the specified topic name and domain ID.

        :param topic_name: The topic name for the DataReader.
        :param domain_id: The DDS domain ID.
        :param qos: Optional QoS settings for the DataReader.
        :return: A configured DataReader instance.
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
        reader = DataReader(self.domain_participant, topic, listener=Listener())
        return reader

    def topic_writer(self, topic_name: str = "", topic_type: Type = None, qos: Qos = None):
        """
        Create a DDS Publisher with the specified topic name and domain ID.

        :param topic_name: The topic name for the DataWriter.
        :param domain_id: The DDS domain ID.
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
