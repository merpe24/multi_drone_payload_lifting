# ros2 launch ~/Projects/multi_drone_payload_lifting/ros2_ws/src/flow_tags.launch.py
# Clock, image, camera_info, detector bridges + apriltag_odom.py for both drone

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

SRC = '/home/premmm/Projects/multi_drone_payload_lifting/ros2_ws/src'
TAGS_YAML = f'{SRC}/tags_36h11.yaml'

# Per-drone config: (instance, namespace, origin_e, isolate_tf)
# Drone 0: default ns, global /tf, no origin offset.
# Drone 1: /px4_1 ns, isolated /px4_1/tf, +2.0 E spawn offset.
DRONES = [
    (0, '',      0.0, False),
    (1, 'px4_1', 2.0, True),
]


def cam(i):
    return (f'/world/flow_world/model/x500_flow_{i}'
            f'/link/camera_link/sensor/camera')


def generate_launch_description():
    actions = []

    # Shared clock bridge (REQUIRED â sim-time timer nodes freeze without it)
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    ))

    for i, ns, origin_e, isolate_tf in DRONES:
        img = f'{cam(i)}/image'
        info = f'{cam(i)}/camera_info'

        # image bridge (gz -> sensor_msgs/Image)
        actions.append(Node(
            package='ros_gz_image', executable='image_bridge',
            name=f'image_bridge_{i}', output='screen', arguments=[img],
        ))
        # camera_info bridge
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'caminfo_bridge_{i}', output='screen',
            arguments=[f'{info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        ))

        # apriltag detector (remap image_rect + camera_info; isolate /tf for d1)
        tag_remaps = [('image_rect', img), ('camera_info', info)]
        if isolate_tf:
            tag_remaps += [('/tf', f'/{ns}/tf'),
                           ('/tf_static', f'/{ns}/tf_static')]
        actions.append(Node(
            package='apriltag_ros', executable='apriltag_node',
            name='apriltag', namespace=ns, output='screen',
            remappings=tag_remaps,
            parameters=[TAGS_YAML, {'use_sim_time': True}],
        ))

    return LaunchDescription(actions)