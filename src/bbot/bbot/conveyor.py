import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Int16

class Conveyor(Node):
    def __init__(self):
        super().__init__('conveyor')

        # Set GPIO pin number (BCM numbering)
        self.gpio_pin = 18  
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        GPIO.output(self.gpio_pin, GPIO.HIGH)

        # Create subscriber to receive Boolean messages
        self.subscription = self.create_subscription(
            Int16,
            'cmd/conveyor',
            self.sub_callback,
            10
        )
        self.get_logger().info('GPIO Controller Node Initialized')

    def sub_callback(self, msg):
        GPIO.output(self.gpio_pin, msg.data)
        self.get_logger().info(f'Turning conveyor ({self.gpio_pin}) {"ON" if (msg.data == 0) else "OFF"}')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Conveyor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
