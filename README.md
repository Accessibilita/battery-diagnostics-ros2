# battery-diagnostics-ros2
ROS2 Package for Accessing Battery BMS data via bluetooth and transmitting battery level/usage to JoyStick hat for display on user LEDs

## Overview

The `WheelChairDiagnostics` ROS 2 package is designed to enable communication with a wheelchair system over the serial port on a Raspberry Pi. It includes a ROS 2 node that reads data from the serial port and publishes it as a ROS 2 message. Additionally, it provides a basic structure for further development and integration into a larger robotic system.

## Features

- Serial communication with a wheelchair system.
- ROS 2 node for publishing diagnostics data.

## Dependencies

- ROS 2 (Foxy or later)
- Python 3
- pyserial

## Installation

1. Make sure you have a working ROS 2 installation on your system. If not, you can follow the ROS 2 installation instructions: [ROS 2 Installation Guide](https://docs.ros.org/en/foxy/Installation.html)

2. Create a new ROS 2 workspace:

    ```bash
    mkdir -p WheelChairDiagnostics_ws/src
    cd WheelChairDiagnostics_ws
    colcon build
    ```

3. Clone the `WheelChairDiagnostics` repository into the `src` directory:

    ```bash
    cd src
    git clone <repository_url>
    ```

4. Install the required Python dependencies:

    ```bash
    pip3 install pyserial
    ```

5. Build the ROS 2 package:

    ```bash
    cd ..
    colcon build
    ```

## Usage

1. Source the ROS 2 setup file:

    ```bash
    source install/setup.bash
    ```

2. Run the `serial_communication_node`:

    ```bash
    ros2 run WheelChairDiagnostics serial_communication_node
    ```

   Adjust the serial port and baudrate in the `serial_communication_node.py` script as needed.

## Node Details

### `serial_communication_node`

- Subscribes to: None
- Publishes to: `/diagnostics_data` (std_msgs.msg.String)

This node reads data from the configured serial port and publishes it as a ROS 2 message on the `/diagnostics_data` topic.

## Contributing

Feel free to contribute to the development of this library. If you find any issues or have suggestions for improvements, please create an issue or submit a pull request.

## License

MIT License

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