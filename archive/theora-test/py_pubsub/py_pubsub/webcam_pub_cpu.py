# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import cv2

# Import the String message type. (Other types are available in the same way.)
from theora_image_transport.msg import Packet

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(Packet, 'topic', 10)

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.video_cap = cv2.VideoCapture(0)
        
        
        # Set video capture properties

    def timer_callback(self):
        msg = Packet()

        success, img = self.video_cap.read()

        if not success:
            self.get_logger().info("Could not read image from camera")
            return
        
        success, buffer = cv2.imencode('.png', img)

        if not success:
            self.get_logger().info("Could not encode image")
            return
        
        msg.data = buffer.tobytes()

        # Publish the message.
        self.publisher_.publish(msg)

        # Log the message.
        # self.get_logger().info('Publishing: "%s"' % msg.data)


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
