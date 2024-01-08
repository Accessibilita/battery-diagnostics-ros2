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

# WheelChairDiagnostics/WheelChairDiagnostics/bluetooth_interface_node.py
import rclpy
from rclpy.node import Node
import serial
import bluetooth
from std_msgs.msg import String

class BluetoothInterfaceNode(Node):
    def __init__(self):
        super().__init__('bluetooth_interface_node')
        self.serial_port = serial.Serial('/dev/ttyS0', 9600, timeout=1)  # Adjust the port and baudrate as needed
        self.bluetooth_server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.bluetooth_server_socket.bind(("", bluetooth.PORT_ANY))
        self.bluetooth_server_socket.listen(1)

        port = self.bluetooth_server_socket.getsockname()[1]
        print(f"Waiting for connection on RFCOMM channel {port}")

        self.client_socket, client_info = self.bluetooth_server_socket.accept()
        print(f"Accepted connection from {client_info}")

        self.publisher = self.create_publisher(String, 'diagnostics_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # Placeholder for sending battery level LED data over UART
        battery_level_led_data = "Battery Level LED Data"  # Replace with your logic

        # Send data over UART
        self.serial_port.write(battery_level_led_data.encode('utf-8'))

        # Read data from the serial port
        serial_data = self.serial_port.readline().decode('utf-8').strip()

        # Placeholder for battery BMS interfacing logic
        bluetooth_data = "Battery BMS Data"  # Replace with your logic

        # Send data over Bluetooth
        self.client_socket.send(bluetooth_data)

        # Publish the received data on the ROS topic
        msg = String()
        msg.data = serial_data
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = BluetoothInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
