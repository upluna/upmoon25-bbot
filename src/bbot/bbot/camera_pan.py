import rcply
from rcply import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Int16



class CameraPan(Node):
    def __init__(self):
        super().__init__('camera_pan')
        

        #set GPIO pin number:
        self.gpio_pin = 19

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        #GPIO.output(self.gpio_pin, GPIO.HIGH)


        # Create subscriber to receive Boolean messages
        self.subscription = self.create_subscription(
            Int16,
            'cmd/camera_pan',
            self.sub_callback,
            10
        )

        self.freq = 50 #50Hz PWM frequency, which is the standard
        self.left_pulse = 2500 #full left 180 degrees
        self.right_pulse = 500 #full right 0 degrees

        self.pwm = GPIO.PWM(self.pwm_pin, self.freq)
        self.pwn.start(0) #no movement at tha beginning

        self.get_logger().info('GPIO Controller Node for Camera Pan Servo Initialized')


    def sub_callback(self, msg):
        if (msg.data == -1): #turn left
            duty_cycle = self.pulse_to_duty(self.left_pulse)
        elif (msg.data == 1): #turn right
            duty_cycle = self.pulse_to_duty(self.right_pulse)
        else:
            duty_cycle = 0 #stop signal send
            self.get_logger().info("stop servo")    

        self.pwm.ChangeDutyCycle(duty_cycle)


    '''this function converts pulse width into duty cycle percent'''
    def pusle_to_duty(self, pulse_us):
        return (pulse_us / 20000) * 100 #convert 500-2500 microseconds to 0-100% duty cycle



    def destroy_node(self):
        self.pwn.stop()
        GPIO.cleanup()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = CameraPan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()