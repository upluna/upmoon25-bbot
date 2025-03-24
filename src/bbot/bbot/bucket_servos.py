import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Int16

# Define the PWM pin
PWM_PIN = 18  # Change this to the actual pin

# Servo PWM Specs
PWM_FREQUENCY = 50  # 50Hz (20ms period)
PWM_PERIOD_US = (1 / PWM_FREQUENCY) * 1000000 # Period in microseconds (us)
PULSE_MIN = 1100  # Fully extended (1.1ms)
PULSE_MAX = 1900  # Fully retracted (1.9ms)
PULSE_RANGE = PULSE_MAX - PULSE_MIN

class BucketServos(Node):
    def __init__(self):
        super().__init__('bucket chain servos')

        # Set GPIO pin number (BCM numbering)
        GPIO.setmode(GPIO.BCM) #research on this GPIO
        GPIO.setup(PWM_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY)

        # Create subscriber to receive position messages
        self.subscription = self.create_subscription(
            Int16,
            'cmd/bucket_pos',
            self.sub_callback,
            10
        )
        self.get_logger().info('GPIO Controller Node Initialized')

    def sub_callback(self, msg):
        self.set_servo_position(msg.data)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

    def set_servo_position(self, position):
        """
        Sets the servo position based on input percentage (0-100%).
        - 0%  -> Fully Extended (1100µs)
        - 100% -> Fully Retracted (1900µs)
        """
        if 0 <= position <= 100:
            pulse_width = PULSE_MIN + (position / 100.0) * PULSE_RANGE
            duty_cycle = (pulse_width / PWM_PERIOD_US) * 100  # Convert to duty cycle
            self.pwm.ChangeDutyCycle(duty_cycle)
            print(f"Servo set to {position}% ({pulse_width}µs pulse)")
        else:
            print("Error: Position must be between 0 and 100.")

def main(args=None):
    rclpy.init(args=args)
    node = BucketServos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
