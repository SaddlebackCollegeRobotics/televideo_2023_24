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
            {'serial_ID': "046d_C922_Pro_Stream_Webcam_F5B8DF8F"},
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
            {'serial_ID': "SunplusIT_Inc_HD_User_Facing"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 30},
        ],
    )

    return LaunchDescription([
        camera1_encoder,
        camera2_encoder,
    ])

