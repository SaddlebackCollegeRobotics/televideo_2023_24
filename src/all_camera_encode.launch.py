from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # CAMERA 1 --------------------------------------------

    camera1_encoder = Node(
        package='camera_pub_sub',
        executable='camera_encoder',
        name='camera1_enc',
        output='screen',
        parameters=[
            {'camera_name': 'camera1'},
            {'serial_ID': "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0003"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 30},
        ],
    )

    # CAMERA 2 --------------------------------------------

    camera2_encoder = Node(
        package='camera_pub_sub',
        executable='camera_encoder',
        name='camera2_enc',
        output='screen',
        parameters=[
            {'camera_name': 'camera2'},
            {'serial_ID': "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0004"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 30},
        ],
    )

    # CAMERA 3 --------------------------------------------

    camera3_encoder = Node(
        package='camera_pub_sub',
        executable='camera_encoder',
        name='camera3_enc',
        output='screen',
        parameters=[
            {'camera_name': 'camera3'},
            {'serial_ID': "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0002"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 30},
        ],
    )

    return LaunchDescription([
        camera1_encoder,
        camera2_encoder,
        camera3_encoder,
    ])

