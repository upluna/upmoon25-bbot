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

from std_msgs.msg import UInt8

class MinimalDriver(Node):

    def __init__(self):
        super().__init__('minimal_driver')

        # How frequently pygame should be polled
        self.clk = 0.01

        self.publisher_ = self.create_publisher(UInt8, 'keyboard', 10)

        # Timer for polling events from pygame
        self.timer = self.create_timer(self.clk, self.pollEvents)

        pygame.init()
        pygame.display.set_mode((300, 300))
        pygame.display.set_caption('bbot driver')

        self.w = 0
        self.s = 0
        self.a = 0
        self.d = 0

        self.FPS = 60

    # Helper method for pollEvents
    def setKeys(self, key, val):
        if (key == 119):
            self.w = val
        if (key == 115):
            self.s = val
        if (key == 97):
            self.a = val
        if (key == 100):
            self.d = val
        
        # Keyboard data is published as a 4 digit binary in integer form, where each
        # digit represents w,s,a,d being down or up: for example,
        # 1011 -> w, a, d are down, s is up
        msg = String()
        msg.data = int(f'0b{self.w}{self.a}{self.s}{self.d}', 2)
        self.publisher_.publish(msg)


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
