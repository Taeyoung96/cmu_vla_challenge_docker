#!/usr/bin/env python
import rosbag
import rospy
from tf2_msgs.msg import TFMessage

def add_namespace_to_tf(tf_msg, namespace):
    # 모든 프레임 이름 수집
    frame_ids = set()
    for transform in tf_msg.transforms:
        frame_ids.add(transform.header.frame_id)
        frame_ids.add(transform.child_frame_id)

    # 프레임 이름에 네임스페이스 추가
    namespaced_frames = {}
    for frame_id in frame_ids:
        # 빈 문자열이 아닌 경우에만 네임스페이스 추가
        if frame_id and not frame_id.startswith(namespace):
            # '/'를 적절히 추가하여 중복을 방지
            namespaced_frame_id = (namespace + '/' + frame_id).replace('//', '/')
            namespaced_frames[frame_id] = namespaced_frame_id
        else:
            namespaced_frames[frame_id] = frame_id

    # 트랜스폼 업데이트
    for transform in tf_msg.transforms:
        transform.header.frame_id = namespaced_frames.get(transform.header.frame_id, transform.header.frame_id)
        transform.child_frame_id = namespaced_frames.get(transform.child_frame_id, transform.child_frame_id)

    return tf_msg




def add_namespace_to_topics(input_bag_path, output_bag_path, namespace):
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            # /tf 및 /tf_static 토픽은 네임스페이스를 추가하지 않음
            if topic == '/tf' or topic == '/tf_static':
                msg = add_namespace_to_tf(msg, namespace)
                outbag.write(topic, msg, t)
                continue

            # 다른 토픽에 대해서는 네임스페이스 추가
            modified_topic = namespace + topic if not topic.startswith(namespace) else topic

            # 메시지를 새로운 토픽으로 재기록
            outbag.write(modified_topic, msg, t)

if __name__ == "__main__":
    input_bag_path = '/root/catkin_ws/src/bag/robot2_2024-01-13-09-49-21.bag'  # Original bag file path (absolute path inside the docker container)
    output_bag_path = '/root/catkin_ws/src/bag/robot2.bag'  # Result bag file path (absolute path inside the docker container)
    namespace = 'robot2'  # Add namespace

    add_namespace_to_topics(input_bag_path, output_bag_path, namespace)
    print("Bag file with added namespace is saved as {}".format(output_bag_path))
