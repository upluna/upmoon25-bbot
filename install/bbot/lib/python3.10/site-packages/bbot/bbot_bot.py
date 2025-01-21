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
from pysabertooth import Sabertooth

saber = Sabertooth('/dev/ttyACM0', baudrate=9600, address=128, timeout=0.1)
saber2 = Sabertooth('/dev/ttyACM1', baudrate=9600, address=128, timeout=0.1)


class KeyboardSubscriber(Node):

    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(
            String,
            'keyboard',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.kb_state = {'w': '0', 's': '0', 'a': '0', 'd': '0', 'throttle' : 100}
        self.get_logger().info('Successful initialization!')

    def listener_callback(self, msg):
        #wasd:throttle
        msg = msg.data
        print(msg)
        self.kb_state['w'] = msg[0]
        self.kb_state['s'] = msg[1]
        self.kb_state['a'] = msg[2]
        self.kb_state['d'] = msg[3]
        self.kb_state['throttle'] = int(msg[5:])

        if (self.kb_state['w'] == '1'):

            t1 = self.kb_state['throttle']
            t2 = self.kb_state['throttle']

            if (self.kb_state['a'] == '1'):
                t2 = int(t2 * 0.5)
            elif (self.kb_state['d'] == '1'):
                t1 = int(t2 * 0.5)

            saber.drive(1, t1)
            saber.drive(2, -t1)
            saber2.drive(1, t2)
            saber2.drive(2, -t2)
            self.get_logger().info("Forward")
        elif (self.kb_state['s'] == '1'):
            saber.drive(1, -self.kb_state['throttle'])
            saber.drive(2, self.kb_state['throttle'])
            saber2.drive(1, -self.kb_state['throttle'])
            saber2.drive(2, self.kb_state['throttle'])

            self.get_logger().info("Backward")
        elif (self.kb_state['a'] == '1'):
            saber.drive(1, -self.kb_state['throttle'])
            saber.drive(2, self.kb_state['throttle'])
        elif (self.kb_state['d'] == '1'):
            saber2.drive(1, -self.kb_state['throttle'])
            saber2.drive(2, self.kb_state['throttle'])

        else:
            saber.stop()
            saber2.stop()
            self.get_logger().info("Pin off")

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
