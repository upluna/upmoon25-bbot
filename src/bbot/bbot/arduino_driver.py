import rclpy
import serial
import serial.tools.list_ports
import subprocess

from rclpy.node import Node
from std_msgs.msg import Float32

class ArduinoDriver(Node):

    def __init__(self):
        super().__init__('arduino_driver')
        self.get_logger().info('Arduino driver initialized')
        self.get_logger().info("Locating arduino port...")

        # Find and flash to Arduino first
        port = self.findArduinoPort()
        if port:
            self.flashHexFile(port, 'SerialTest.ino.hex') 
        else:
            self.get_logger().error("Could not find arduino port!")
            self.destroy_node()

        # Flash succesful: now initialize publishers
        self.sensor_pubs = [
            self.create_publisher(Float32, '/sensor/ir/right', 1),
            self.create_publisher(Float32, '/sensor/ir/left', 1),
            # Not sure what data type we'll use for these
            self.create_publisher(Float32, '/sensor/force/right', 1),
            self.create_publisher(Float32, '/sensor/force/left', 1)
        ]

        # Now listen for serial input from arduino
        ser = serial.Serial(port, 115200, timeout=1)

        while True:
            datum = ser.readline().decode('utf-8').rstrip('\n').split('#')
            # For now, only the left IR sensor is implemented, so there is only one data point
            for i, data in enumerate(datum):
                # Don't know why this happens, strip() doesn't solve for some reason
                if data == '':
                    continue
                msg = Float32()
                msg.data = float(data)
                self.sensor_pubs[i].publish(msg);


    def flashHexFile(self, port, hex_file):
        avrdude_cmd = [
            'avrdude',
            '-v',
            '-V',
            '-c',
            'arduino',
            '-patmega328p',
            'carduino',
            f'-P{port}',
            '-b115200',
            '-D',
            f'-Uflash:w:/home/upmoon25/Documents/robotics/upmoon25-bbot/src/bbot/bbot/arduino/hex/{hex_file}:i'
        ]

        result = subprocess.run(avrdude_cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info("Flash succesful")
        else:
            self.get_logger().error("Flash failed!")
            self.get_logger().error(result)
            self.destroy_node()

    def findArduinoPort(self, arduino_vid='2341', arduino_pid='0043'):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid is not None and port.pid is not None:
                if f'{port.vid:04x}' == arduino_vid and f'{port.pid:04x}' == arduino_pid:
                    return str(port.device)
        return None




def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
