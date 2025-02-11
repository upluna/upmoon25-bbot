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
from std_msgs.msg import Int16


class MinimalDriver(Node):

    def __init__(self):
        super().__init__('minimal_driver')

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
        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 30)	

        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('bbot driver')

        self.throttle = 100.0

        self.FPS = 60

        self.updateThrottleText()

        self.get_logger().info("Succesful initialization")

    def updateThrottleText(self):
        text_surface = self.font.render(f'Throttle: {self.throttle}', False, (255, 255, 255))
        self.screen.fill((0,0,0))
        self.screen.blit(text_surface, (0,0)) 


    # Helper method for pollEvents
    def setKeys(self, key, val):
        velocity_msg = Twist()

        if (key == 119): # w
            velocity_msg.linear.x = self.throttle # set the direction to positive
            self.w = val
        if (key == 115): # s
            velocity_msg.linear.x = -self.throttle # set the throttle to negative
            self.s = val
        if (key == 97): # a
            velocity_msg.angular.z = -1.0 # negative value indicates a left turn
            self.a = val
        if (key == 100): # d
            velocity_msg.angular.z = 1.0 # positive value indicates a right turn
            self.d = val
        if (key == 101 and val == 1):
            self.throttle += 10.0
            if (self.throttle > 100.0):
                self.throttle = 100.0
            self.updateThrottleText()
        if (key == 113 and val == 1):
            self.throttle -= 10.0
            if (self.throttle < 0):
                self.throttle = 0
            self.updateThrottleText()

        # Velocity data is published as a twist object. The linear x component represents throttle (+/- = forward/backward)
        # The angular z component represents turning (+/- = right/left, 0 = no turn). Turning speed is constant regardless of z magnitude
        self.velocity_pub.publish(velocity_msg)


    # Publishes keyboard inputs
    def pollEvents(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                self.destroy_node()
                rclpy.shutdown()

            elif event.type == KEYDOWN:
                key = event.__dict__['key']
                self.setKeys(key, 1)

            elif (event.type == KEYUP):
                key = event.__dict__['key']
                self.setKeys(key, 0)

            pygame.display.update()
            pygame.time.Clock().tick(60)


def main(args=None):
    rclpy.init(args=args)

    minimal_driver = MinimalDriver()

    rclpy.spin(minimal_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
