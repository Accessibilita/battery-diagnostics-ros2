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
