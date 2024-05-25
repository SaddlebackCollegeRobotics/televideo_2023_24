import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64MultiArray
from serial import Serial, SerialException


class PanTiltDriver(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('servo_driver')

        try:
            self.serial = Serial('/dev/ttyUSB0', 9600)
        except SerialException:
            print("Error: Could not open serial port!")
            exit(0)

        self.subscription = self.create_subscription(
            Int64MultiArray,
            '/telecom/pan_tilt_control',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        serial_input = str(msg.data[0]) + ',' + str(msg.data[1]) + '\0'
        self.serial.write(serial_input.encode())


def main(args=None):
    rclpy.init(args=args)

    pan_tilt_driver = PanTiltDriver()

    rclpy.spin(pan_tilt_driver)

    pan_tilt_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
