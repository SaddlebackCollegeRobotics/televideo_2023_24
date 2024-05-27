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

from std_msgs.msg import String
import cv2
import numpy
import io

from theora_image_transport.msg import Packet



class MinimalSubscriber(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_subscriber')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            Packet,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    # This callback definition simply prints an info message to the console, along with the data it received. 
    def listener_callback(self, msg):

        # Convert the image to a numpy array
        buffer = numpy.frombuffer(msg.data, dtype=numpy.uint8)

        # Decode the image
        img = cv2.imdecode(buffer, flags=cv2.IMREAD_COLOR)

        # Process the image
        self.process_image(img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit()
        

    def process_image(self, image):
        # Example: Display the image using OpenCV
        cv2.imshow("Received Image", image)


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
