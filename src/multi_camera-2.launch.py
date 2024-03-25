from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_1 = Node(
        package='py_pubsub',
        executable='talker',
        name='camera_4',
        output='screen',
        parameters=[
            {'device_path': '/dev/video6'},
            {'topic_name': '/camera_4/compressed'},
        ],
    )

    return LaunchDescription([
        camera_1,
    ])

