import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class HeartbeatPublisher(Node):

    def __init__(self):

        super().__init__('heartbeat_publisher')

        self.heartbeat_publisher = self.create_publisher(Empty, '/system/heartbeat', 10)

        self.HEARTBEAT_PERIOD = 0.5
        self.timer = self.create_timer(self.HEARTBEAT_PERIOD, self.timer_callback)

        self.msg = Empty()

    def timer_callback(self):
        self.heartbeat_publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    heartbeat_publisher = HeartbeatPublisher()

    rclpy.spin(heartbeat_publisher)

    heartbeat_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
