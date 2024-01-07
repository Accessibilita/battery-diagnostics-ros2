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
