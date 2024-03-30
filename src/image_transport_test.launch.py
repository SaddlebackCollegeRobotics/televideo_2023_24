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
            {'serial_ID': "SunplusIT_Inc_HD_User_Facing"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 30},
        ],
    )

    camera_decoder = Node(
        package='camera_pub_sub',
        executable='camera_decoder',
        name='camera1_dec',
        output='screen',
        parameters=[
            {"camera_name": "camera1"},
            {"image_transport": "compressed"},
        ],
    )

    return LaunchDescription([
        camera_encoder,
        camera_decoder,
    ])

