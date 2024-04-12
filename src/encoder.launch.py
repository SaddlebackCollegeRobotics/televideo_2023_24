from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_encoder = Node(
        package='camera_pub_sub',
        executable='camera_encoder',
        name='camera1_enc',
        output='screen',
        parameters=[
            {'camera_name': 'camera1'},
            {'serial_ID': "Technologies__Inc._ZED_2i_OV0001"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 30},
        ],
    )

    return LaunchDescription([
        camera_encoder
    ])

