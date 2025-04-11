#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('bld515c_motor_controller')

        # Modbus config
        self.client = ModbusClient(
            method='rtu',
            port='/dev/ttyUSB0',  # Update if different
            baudrate=9600,
            stopbits=1,
            bytesize=8,
            parity='N',
            timeout=1
        )
        self.slave_id = 1  # Modbus address of the motor controller

        # Motor control registers
        self.REG_CONTROL = 0x8106
        self.REG_SPEED = 0x8110

        # Init Modbus connection
        if not self.client.connect():
            self.get_logger().error('Failed to connect to motor controller.')
            rclpy.shutdown()
            return

        # Subscribe to RPM commands
        self.subscriber = self.create_subscription(
            Int16,
            'cmd/bucket_vel',
            self.rpm_callback,
            10
        )
        self.get_logger().info('Bucket Chain Motor controller node initialized.')

    def rpm_callback(self, msg: Int16):
        rpm = msg.data
        try:
            if rpm == 0:
                self.disable_motor()
            else:
                direction = 1 if rpm < 0 else 0  # 1 = reverse, 0 = forward
                self.set_motor_control(enable=True, direction=direction)
                self.set_motor_speed(abs(rpm))
        except ModbusIOException as e:
            self.get_logger().error(f'Modbus IO error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    def set_motor_control(self, enable: bool, direction: int):
        # Build control byte:
        # Bit 0 = EN (1: enable)
        # Bit 1 = FR (1: reverse)
        control_byte = 0
        if enable:
            control_byte |= 0x01  # Enable motor
        if direction == 1:
            control_byte |= 0x02  # Reverse

        #control_word = (0x04 << 8) | control_byte  # second byte = 0x04 = internal mode
        control_word = (0x03 << 8) | control_byte  # Internal control + internal speed mode?

        result = self.client.write_register(self.REG_CONTROL, control_word, unit=self.slave_id)

        if result.isError():
            self.get_logger().warn(f'Failed to write motor control: {control_word:#04x}')
        else:
            self.get_logger().info(
                f'Motor {"enabled" if enable else "disabled"} | Direction: {"reverse" if direction else "forward"}')

    def disable_motor(self):
        control_word = (0x04 << 8) | 0x00  # Disable, keep mode = internal
        result = self.client.write_register(self.REG_CONTROL, control_word, unit=self.slave_id)

        if result.isError():
            self.get_logger().warn('Failed to disable motor')
        else:
            self.get_logger().info('Motor disabled (RPM = 0)')

    def set_motor_speed(self, rpm: int):
        result = self.client.write_register(self.REG_SPEED, rpm, unit=self.slave_id)
        if result.isError():
            self.get_logger().warn(f'Failed to set speed to {rpm} RPM')
        else:
            self.get_logger().info(f'Set speed: {rpm} RPM')

    def destroy_node(self):
        self.client.close()
        self.get_logger().info('Closed Modbus connection.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
