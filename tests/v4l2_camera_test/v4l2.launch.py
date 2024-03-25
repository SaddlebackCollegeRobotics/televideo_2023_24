from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    v4l2_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[
            {'image_size': [1280, 720]},
            {'video_device': '/dev/video2'},
            # {'pixel_format': 'YUYV'},
            # {'output_encoding': 'rgb'},
            #{'camera_info_url': 'file:///home/cameron/workspaces/v4l2_camera_test/ost.yaml'}
            # {'ffmpeg_image_transport.encoding': 'hevc_nvenc'},
            # {'ffmpeg_image_transport.profile': 'main'},
            # {'ffmpeg_image_transport.preset': 'll'},
            # {'ffmpeg_image_transport.gop_size': 15},
        ]
    )


    return LaunchDescription([
        v4l2_camera,
    ])
