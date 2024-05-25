import rclpy
from rclpy.node import Node
from numpy import clip

from std_msgs.msg import Int64MultiArray
from ament_index_python.packages import get_package_share_directory

from . import gamepad_input as gmi


class ControlsPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('controls_publisher')

        self.publisher_ = self.create_publisher(Int64MultiArray, '/telecom/pan_tilt_control', 1)

        self.msg = Int64MultiArray()

        self.current_pitch = 90
        self.current_yaw = 90
        self.speed_factor = 3

        self.pitch_range = (51, 184)
        self.yaw_range = (5, 183)

        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.AXIS_DEADZONE = 0.1
        gmi.setConfigFile(get_package_share_directory('pan_tilt_camera') + '/gamepads.config')
        gmi.run_event_loop()

    def timer_callback(self):

        gamepad = gmi.getGamepad(0)

        # Trigger press acts as a safety switch as well as a workaround
        # for a pygame bug that causes specifically 8bitdo controllers to
        # register a constant input of 1.0 and -1.0 on joystick axes, when
        # unplugged and plugged back in.
        # Trigger also acts as a safety button for system.
        if gamepad != None and gmi.getTriggers(gamepad, self.AXIS_DEADZONE)[1] > 0:
            (x_hat, y_hat) = gmi.getHat(gamepad)

            self.current_pitch += -y_hat * self.speed_factor
            self.current_yaw += x_hat * self.speed_factor

            self.current_pitch = clip(self.current_pitch, self.pitch_range[0], self.pitch_range[1])
            self.current_yaw = clip(self.current_yaw, self.yaw_range[0], self.yaw_range[1])

            self.msg.data = [int(self.current_pitch), int(self.current_yaw)]

            self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    controls_publisher = ControlsPublisher()

    rclpy.spin(controls_publisher)

    controls_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
