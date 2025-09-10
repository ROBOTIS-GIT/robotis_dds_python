import time
import math
from cyclonedds.core import Qos, Policy
from cyclonedds.util import duration

from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_, JointTrajectoryPoint_
from robotis_dds_python.idl.builtin_interfaces.msg import Time_, Duration_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.tools.topic_manager import TopicManager


qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)

# Use the utility function to create the writer
topic_manager = TopicManager()
writer = topic_manager.topic_writer(topic_name='/joint_trajectory', topic_type=JointTrajectory_, qos=qos)


def create_trajectory(t: float) -> JointTrajectory_:
    joint_names = ["j1", "j2", "j3"]
    points = []

    for i in range(5):
        sec = i + 1
        positions = [math.sin(t + i), math.cos(t + i), math.sin(t * 0.5 + i)]
        point = JointTrajectoryPoint_(
            positions=positions,
            velocities=[],
            accelerations=[],
            effort=[],
            time_from_start=Duration_(sec=sec, nanosec=0)
        )
        points.append(point)

    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)

    header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="base_link")

    return JointTrajectory_(header=header, joint_names=joint_names, points=points)


t = 0.0

while True:
    t += 0.1
    msg = create_trajectory(t)
    writer.write(msg)
    print(f"Published {len(msg.points)} points")
    time.sleep(1.0)
