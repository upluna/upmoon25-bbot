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
from geometry_msgs.msg import Twist
from pysabertooth import Sabertooth

saber = Sabertooth('/dev/ttyACM0', baudrate=9600, address=128, timeout=0.1) #left wheels
saber2 = Sabertooth('/dev/ttyACM1', baudrate=9600, address=128, timeout=0.1) #right wheels


class DriveMotors(Node):

    def __init__(self):
        super().__init__('drive_motors')
        self.subscription = self.create_subscription(
            Twist,
            'cmd/velocity',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Successful initialization!')

    def listener_callback(self, msg):
        #wasd:throttle
        print(msg)
        throttle = msg.linear.x
        rotation = msg.angular.z

        if (throttle > 0):

            t1 = throttle
            t2 = throttle

            if (rotation < 0.0): # left turn
                t2 = int(t2 * 0.5)
            elif (rotation > 0.0): # right turn
                t1 = int(t2 * 0.5)

            saber.drive(1, t1)
            saber.drive(2, -t1)
            saber2.drive(1, t2)
            saber2.drive(2, -t2)
            self.get_logger().info("Forward")
        elif (throttle > 0.0):
            saber.drive(1, -throttle)
            saber.drive(2, self.kb_state['throttle'])
            saber2.drive(1, -self.kb_state['throttle'])
            saber2.drive(2, self.kb_state['throttle'])

            self.get_logger().info("Backward")
        elif (rotation < 0.0): # left
            saber.drive(1, -throttle)
            saber.drive(2, throttle)
        elif (rotation > 0.0): # right
            saber2.drive(1, -throttle)
            saber2.drive(2, throttle)

        else:
            saber.stop()
            saber2.stop()
            self.get_logger().info("Stopped")



def main(args=None):
    
    rclpy.init(args=args)

    drive_motors = DriveMotors()

    rclpy.spin(drive_motors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
