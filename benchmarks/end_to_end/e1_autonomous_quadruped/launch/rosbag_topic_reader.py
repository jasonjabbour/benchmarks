from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

reader = SequentialReader()
reader.open(
    StorageOptions(uri="/tmp/benchmark_ws/src/rosbags/end_to_end/rosbag2_end_to_end_1", storage_id="sqlite3"),
    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
)

topics = reader.get_all_topics_and_types()
topic_types = {topic.name: topic.type for topic in topics}

frame_count = 0
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == "/velodyne_points":
        msg = deserialize_message(data, PointCloud2)
        pc = np.array(list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)))
        np.save(f"velodyne_frame_{frame_count:04d}.npy", pc)
        frame_count += 1

print(f"Extracted {frame_count} frames from ROS 2 bag.")