from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_1 = Node(
        package='py_pubsub',
        executable='talker',
        name='camera_1',
        output='screen',
        parameters=[
            {'device_path': '/dev/video2'},
            {'topic_name': '/camera_1/compressed'},
        ],
    )

    camera_2 = Node(
        package='py_pubsub',
        executable='talker',
        name='camera_2',
        output='screen',
        parameters=[
            {'device_path': '/dev/video4'},
            {'topic_name': '/camera_2/compressed'},
        ],
    )

    camera_3 = Node(
        package='py_pubsub',
        executable='talker',
        name='camera_3',
        output='screen',
        parameters=[
            {'device_path': '/dev/video6'},
            {'topic_name': '/camera_3/compressed'},
        ],
    )

    camera_4 = Node(
        package='py_pubsub',
        executable='talker',
        name='camera_4',
        output='screen',
        parameters=[
            {'device_path': '/dev/video8'},
            {'topic_name': '/camera_4/compressed'},
        ],
    )
    

    return LaunchDescription([
        camera_1,
        camera_2,
        camera_3,
        camera_4,
    ])

