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
            {'serial_ID': "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0001"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 20},
            {'camera1.transport.format': 'jpeg'},
            {'camera1.transport.jpeg_quality': 10}
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
            {'serial_ID': "16MP_Camera_Mamufacture_16MP_USB_Camera_2022050701"},
            {'image_width': 1280},
            {'image_height': 720},
            {'camera_fps': 20},
            {'camera2.transport.format': 'jpeg'},
            {'camera2.transport.jpeg_quality': 10}
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
            {'camera_fps': 20},
            {'camera3.transport.format': 'jpeg'},
            {'camera3.transport.jpeg_quality': 10}
        ],
    )

    return LaunchDescription([
        camera1_encoder,
        camera2_encoder,
        camera3_encoder,
    ])

