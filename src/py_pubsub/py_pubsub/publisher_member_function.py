import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data


import cv2
from cv_bridge import CvBridge

from time import sleep


class CameraPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('camera_publisher')

        self.declare_parameter('device_path', '/dev/video0')
        self.declare_parameter('image_size', (1920, 1080))
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('compression_format', 'MJPG')
        self.declare_parameter('topic_name', '/image_raw/compressed')

        device_path = self.get_parameter('device_path').get_parameter_value().string_value
        image_size = self.get_parameter('image_size').get_parameter_value().integer_array_value
        camera_fps = self.get_parameter('camera_fps').get_parameter_value().integer_value
        compression_format = self.get_parameter('compression_format').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(CompressedImage, topic_name, qos_profile_sensor_data)

        self._bridge = CvBridge()

        # Define the GStreamer pipeline for the camera
        gstreamer_api=f"v4l2src device={device_path} ! \
            image/jpeg,\
            width={image_size[0]},\
            height={image_size[1]},\
            framerate={camera_fps}/1,\
            format=(string){compression_format} ! \
            decodebin ! appsink"

        self.cap = cv2.VideoCapture(gstreamer_api, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            exit(1)

        timer_period = 1/camera_fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):

        msg = CompressedImage()

        success, frame = self.cap.read()

        if success:
            msg = self._bridge.cv2_to_compressed_imgmsg(frame)
            print("Publishing frame")
            self.publisher_.publish(msg)
        else:
            print("Error reading frame")
            sleep(1)


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
