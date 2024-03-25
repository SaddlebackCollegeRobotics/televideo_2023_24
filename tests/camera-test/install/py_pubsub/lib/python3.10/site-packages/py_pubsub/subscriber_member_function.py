import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

# import senson qos
from rclpy.qos import qos_profile_sensor_data

class MinimalSubscriber(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_subscriber')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        WINDOW_SIZE = (1920, 1080)

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', WINDOW_SIZE[0], WINDOW_SIZE[1])

        self._bridge = CvBridge()


    def listener_callback(self, msg):

        frame = self._bridge.compressed_imgmsg_to_cv2(msg)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
