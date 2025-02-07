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
import pygame

from rclpy.node import Node

from pygame.locals import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int16

class JoystickDriver(Node):

    def __init__(self):
        super().__init__('joystick_driver')

        # How frequently pygame should be polled
        self.clk = 0.01

        self.velocity_pub = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.conveyor_pub = self.create_publisher(Int16, 'cmd/conveyor', 10)
        self.bucket_vel_pub = self.create_publisher(Int16, 'cmd/bucket_vel', 10)
        self.bucket_pos_pub = self.create_publisher(Int16, 'cmd/bucket_pos', 10)
        self.tensioner_pub = self.create_publisher(Int16, 'cmd/tensioner', 10)

        # Timer for polling events from pygame
        self.timer = self.create_timer(self.clk, self.pollEvents)

        pygame.init()
        pygame.joystick.init()

        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('testbot driver')

        self.controller = pygame.joystick.Joystick(0)

        self.FPS = 25

    # Publishes joystick inputs
    def pollEvents(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                self.destroy_node()
                rclpy.shutdown()
                pygame.quit()

        throttle = round(-100 * self.controller.get_axis(1))
        if (throttle <= 5 and throttle >= -5):
            throttle = 0
        
        rotation = round(100 * self.controller.get_axis(3))
        if (rotation <= 5 and rotation >= -5):
            rotation = 0

        # Publish joystick data
        msg = Twist()
        msg.linear.x = throttle
        msg.angular.z = rotation
        self.velocity_pub.publish(msg)

        pygame.display.update()
        pygame.time.Clock().tick(self.FPS)


def main(args=None):
    rclpy.init(args=args)

    joystick_driver = JoystickDriver()

    rclpy.spin(joystick_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
