"""
MIT License

Copyright (c) 2024 Adam Vadala-Roth & Mobility Indepence Foundation

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# WheelChairDiagnostics/WheelChairDiagnostics/serial_communication_node.py
import rclpy
from rclpy.node import Node
import serial

class SerialCommunicationNode(Node):
    def __init__(self):
        super().__init__('serial_communication_node')
        self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=1)  # Adjust the port and baudrate as needed

        self.publisher = self.create_publisher(String, 'diagnostics_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # Read data from the serial port
        data = self.serial_port.readline().decode('utf-8').strip()

        # Publish the received data
        msg = String()
        msg.data = data
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SerialCommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
