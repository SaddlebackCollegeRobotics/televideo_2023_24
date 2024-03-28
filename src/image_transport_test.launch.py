from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_publisher = Node(
        package='camera_pub_sub',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[
            {'device_index': 2},
            {'image_width': 1920},
            {'image_height': 1080},
            {'camera_fps': 30},
            {'base_topic': '/camera/image'}
        ],
    )

    camera_subscriber = Node(
        package='camera_pub_sub',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
        parameters=[
            {'windows_name': 'camera viewer'},
            {'base_topic': '/camera/image'},
            {'image_transport': 'compressed'}
        ],
    )

    return LaunchDescription([
        camera_publisher,
        camera_subscriber
    ])

