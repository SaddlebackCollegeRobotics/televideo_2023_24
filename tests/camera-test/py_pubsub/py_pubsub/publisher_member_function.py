import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from time import sleep
# Import the String message type. (Other types are available in the same way.)
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(CompressedImage, '/image_raw/compressed', qos_profile_sensor_data)

        DEVICE_PATH = '/dev/video2'
        FPS = 30
        IMAGE_SIZE = (1280, 720)

        self._bridge = CvBridge()

        # Define the GStreamer pipeline for the camera
        camSet="v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1,format=(string)MJPG ! decodebin ! appsink"

        # Open the camera
        self.cap = cv2.VideoCapture(camSet, cv2.CAP_GSTREAMER)

        sleep(2)

        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_SIZE[0])
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_SIZE[1])
        # self.cap.set(cv2.CAP_PROP_FPS, FPS)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 1/FPS  # seconds
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

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
