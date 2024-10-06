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

from std_msgs.msg import UInt8


class KeyboardSubscriber(Node):

    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(
            UInt8,
            'keyboard',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.kb_state = {'w': '0', 's': '0', 'a': '0', 'd': '0'}

    def listener_callback(self, msg):
        bin_string = bin(int(msg.data))[2:].zfill(4)
        self.kb_state['w'] = bin_string[0]
        self.kb_state['s'] = bin_string[1]
        self.kb_state['a'] = bin_string[2]
        self.kb_state['d'] = bin_string[3]
        self.get_logger().info('Keyboard: "%s"' % self.kb_state)


def main(args=None):
    rclpy.init(args=args)

    keyboard_subscriber = KeyboardSubscriber()

    rclpy.spin(keyboard_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
