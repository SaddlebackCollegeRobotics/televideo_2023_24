from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # CAMERA 1 --------------------------------------------

    camera1_decoder = Node(
        package='camera_pub_sub',
        executable='camera_decoder',
        name='camera1_dec',
        output='screen',
        parameters=[
            {"camera_name": "camera1"},
            {"image_transport": "compressed"},
        ],
    )

    camera1_viewer = Node(
        package='camera_pub_sub',
        executable='camera_viewer',
        name='camera1_view',
        output='screen',
        parameters=[
            {'camera_name': 'camera1'},
            {'window_width': 960},
            {'window_height': 540}
        ],
    )

    # CAMERA 2 --------------------------------------------

    camera2_decoder = Node(
        package='camera_pub_sub',
        executable='camera_decoder',
        name='camera2_dec',
        output='screen',
        parameters=[
            {"camera_name": "camera2"},
            {"image_transport": "compressed"},
        ],
    )

    camera2_viewer = Node(
        package='camera_pub_sub',
        executable='camera_viewer',
        name='camera2_view',
        output='screen',
        parameters=[
            {'camera_name': 'camera2'},
            {'window_width': 960},
            {'window_height': 540}
        ],
    )

    # CAMERA 3 --------------------------------------------

    camera3_decoder = Node(
        package='camera_pub_sub',
        executable='camera_decoder',
        name='camera3_dec',
        output='screen',
        parameters=[
            {"camera_name": "camera3"},
            {"image_transport": "compressed"},
        ],
    )

    camera3_viewer = Node(
        package='camera_pub_sub',
        executable='camera_viewer',
        name='camera3_view',
        output='screen',
        parameters=[
            {'camera_name': 'camera3'},
            {'window_width': 960},
            {'window_height': 540}
        ],
    )

    return LaunchDescription([
        camera1_decoder,
        camera1_viewer,
        camera2_decoder,
        camera2_viewer,
        #camera3_decoder,
        #camera3_viewer,
    ])

