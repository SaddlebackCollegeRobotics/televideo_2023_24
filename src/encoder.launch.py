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
            {'serial_ID': "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0001"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 30},

            {'camera1.transport.format': 'jpeg'},
            {'camera1.transport.jpeg_quality': 10}


        ],
    )

    return LaunchDescription([
        camera_encoder
    ])

