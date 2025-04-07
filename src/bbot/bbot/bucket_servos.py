import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Int16

# Define the PWM pin
PWM_PIN = 18  # Change this to the actual pin

# Servo PWM Specs
PWM_FREQUENCY = 50  # 50Hz (20ms period)
MIN_DC = 5.5
MAX_DC = 9.5
MAX_RANGE = 100
MIN_RANGE = 0
INIT_RANGE = 0

class BucketServos(Node):
    def __init__(self):
        super().__init__('bucket_servos')

        # Set GPIO pin number (BCM numbering)
        try:
            GPIO.setmode(GPIO.BCM)
        except Exception:
            print('GPIO failure')
        try:
            GPIO.setup(PWM_PIN, GPIO.OUT, initial = GPIO.LOW)
        except Exception:
            print('Pin setup failure')

        self.pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY)
        self.pwm.start(self.convertRangeToDutyCycle(INIT_RANGE))

        # Create subscriber to receive position messages
        self.subscription = self.create_subscription(
            Int16,
            'cmd/bucket_pos',
            self.sub_callback,
            10
        )
        self.get_logger().info('GPIO Controller Node Initialized')

    def sub_callback(self, msg):
        self.pwm.ChangeDutyCycle(self.convertRangeToDutyCycle(msg.data))

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

    def convertRangeToDutyCycle(self, percent):
        if (percent < MIN_RANGE or percent > MAX_RANGE):
            print('Servo set out of bounds')
            percent = INIT_RANGE
        dc = (percent * (MAX_DC - MIN_DC) / MAX_RANGE) + MIN_DC
        print('Setting servo to %d' % (dc))
        return dc

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
