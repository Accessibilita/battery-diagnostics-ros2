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

# WheelChairDiagnostics/WheelChairDiagnostics/diagnostics_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from WheelChairDiagnostics.serial_communication_node import SerialCommunicationNode
from WheelChairDiagnostics.bluetooth_interface_node import BluetoothInterfaceNode

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        # Create instances of SerialCommunicationNode and BluetoothInterfaceNode
        self.serial_communication_node = SerialCommunicationNode()
        self.bluetooth_interface_node = BluetoothInterfaceNode()

        # Create a timer to execute the update method at a regular interval
        self.timer = self.create_timer(1.0, self.update)

    def update(self):
        # Place any common logic or additional logic here that you want to execute periodically

        # For now, let's print a message
        self.get_logger().info('DiagnosticsNode update')

def main():
    rclpy.init()

    # Create an instance of DiagnosticsNode
    node = DiagnosticsNode()

    # Spin the node to keep it running
    rclpy.spin(node)

    # Destroy the node when the spinning is stopped
    node.destroy_node()

    # Shutdown the ROS context
    rclpy.shutdown()

if __name__ == '__main__':
    main()
