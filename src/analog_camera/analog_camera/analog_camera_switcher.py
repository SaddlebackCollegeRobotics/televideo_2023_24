
import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from rcl_interfaces.srv import DescribeParameters


class AnalogCameraSwitcher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('analog_camera_switcher')

        try:
            self.serial = Serial('/dev/ttyS0', 9600)
        except SerialException:
            print("Error: Could not open serial port!")
            exit(0)

        self.service = self.create_service(
            DescribeParameters.Request,
            '/analog/switch_camera',
            self.listener_callback,
            10)

    def listener_callback(self, msg: DescribeParameters.Request):
        serial_input = str(msg.names[0] + msg.names[1])
        self.serial.write(serial_input.encode())


def main(args=None):
    rclpy.init(args=args)

    analog_camera_switcher = AnalogCameraSwitcher()

    rclpy.spin(analog_camera_switcher)

    analog_camera_switcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
