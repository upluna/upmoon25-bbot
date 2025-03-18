import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Int16



class CameraHeight(Node):
    def __init__(self):
        super().__init__('camera_height')

        self.gpio_pin = 40 # pin 40 is a PWM pin, refer to PWM_pins.txt
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        GPIO.output(self.gpio_pin, GPIO.HIGH)

        # Create subscriber to receive Integer16 messages
        self.subscription = self.create_subscription(
            Int16,
            'cmd/camera_height',
            self.sub_callback,
            10
        )
        self.get_logger().info('GPIO controller node for Camera Actuator Initialized')


    def sub_callback(self, msg):
        if (msg.data == 1): #move the camera height up
            GPIO.output(self.gpio_pin, GPIO.LOW)
        else: #don't move the camera height at all #TODO: Implement the height of camera moving down as well.
            GPIO.output(self.gpio_pin, GPIO.HIGH)
        self.get_logger().info(f'Extending camera height mount with an actuator ({self.gpio_pin}) {"ON" if (msg.data == 1) else "OFF"}')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()



    def main(args=None):
        rclpy.init(args=args)
        node = CameraHeight()
        try:
            rclpy.spin(node) #.spin() also works on actuators
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
