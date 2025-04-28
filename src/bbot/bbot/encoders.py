#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO
import time

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')

        # GPIO pin configuration
        self.pin_a = 17  # BCM numbering
        self.pin_b = 18

        # Encoder state
        self.position = 0
        self.last_a = 0
        self.last_b = 0

        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._update_encoder)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._update_encoder)

        # Publisher
        self.encoder_pub = self.create_publisher(Int32, 'encoder_count', 10)

        # Timer to publish at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info("EncoderReader node started.")

    def _update_encoder(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)

        # Determine direction
        if a != self.last_a or b != self.last_b:
            if a == b:
                self.position += 1
            else:
                self.position -= 1
            self.last_a = a
            self.last_b = b

    def publish_position(self):
        msg = Int32()
        msg.data = self.position
        self.encoder_pub.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
