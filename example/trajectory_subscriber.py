from cyclonedds.core import Listener, Qos, Policy
from cyclonedds.util import duration

from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_
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
reader = topic_manager.topic_reader(topic_name='/joint_trajectory', topic_type=JointTrajectory_, qos=qos)

while True:
    try:
        for sample in reader.take_iter():
            print(sample)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pass
