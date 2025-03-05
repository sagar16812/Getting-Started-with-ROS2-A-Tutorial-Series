# Getting Started with ROS2

Welcome to the official repository for the **[Getting Started with ROS2](https://medium.com/@sagarcadet/list/getting-started-with-ros2-adb24ab6d8dd)** article series on Medium. This repository contains multiple guides and example packages for working with ROS 2.

## Guides and Documentation
- **installation_guide/**: Step-by-step instructions for installing ROS 2 on a desktop, laptop, or Raspberry Pi, along with environment setup guidelines.
- **ros2_multi-machine_communication_guide/**: A comprehensive guide on configuring ROS 2 for multi-machine communication.
- **raspi_headless_camera_calibration_guide/**: Instructions for performing camera calibration on a Raspberry Pi without a graphical user interface.
- **ros2_bag_recording_and_playback_guide/**: Guidelines on recording and replaying ROS 2 bag files for data analysis and debugging.
- **Rviz2_guide/**: A guide to using Rviz2 for visualizing ROS 2 data from active nodes or recorded rosbag files.

## List of example Packages:
- **simple_node_example**: A basic example demonstrating a simple ROS 2 node.
- **pub_sub_tutorial**: Illustrates the implementation of publisher and subscriber nodes for ROS 2 topics.
- **battery_status**: Demonstrates client-server node communication in ROS 2.
- **rotate_turtle_action_client**: Implements an action client node to control the movement of the Turtlesim TurtleBot.

## Raspberry Pi Image with Ubuntu Server 22.04 and ROS2 Humble Base + Perception

We have created a Raspberry Pi image with **Ubuntu Server 22.04 for arm64 and ROS2 base + perception** pre-installed to help you get started quickly. This image can be downloaded directly from this [link](https://drive.google.com/file/d/1yvv7u4Z7PgbNEOfC1JUykrVPpexAQosP/view?usp=sharing), flash it to a SD card, and boot on a Raspberry Pi 3 or above model. In case, you are not able to use this image, you can install ROS2 on Raspberry Pi by checking the `installation_guide` directory or following this [link](https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series/tree/main/installation_guide).

### How to Use the Image
Before you start, you need an SD card that is at least 16GB in size.

1. **Download the Image File**:
   Download the compressed image file from the link above.

2. **Extract the Image File**:
   The image is compressed with the `.gz` compression format. You will need to decompress it. Once decompressed, you should have a file that ends with the extension `.img`. On windows, try using 7-zip. On Mac/GNU-Linux, use `gunzip ubuntu-server-22.04-ros2-base.img.gz`

3. **Write the Image to an SD Card**:
   Use a tool like [Etcher](https://etcher.balena.io/) or [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to write the extracted `.img` file to an SD card.

4. **Insert the SD Card into Your Raspberry Pi**:
   Unplug the SD card and plug it into your Raspberry Pi and power it on. Wait for atleast 5 minutes for the first boot.

5. **Get your Raspberry Pi's IP Address**:
    Make sure your System and Raspberry Pi is connected to the same Wi-Fi network. Try to ping the raspberry pi with the hostname. 
    ```bash
    ping raspberrypi.local
    ```
    and get the ip address of Pi.

6. **Access Your Raspberry Pi**:
   The default username is `ubuntu` and the password is `6496`. The default hostname is `raspberrypi`. You can login either directly on the Raspberry Pi or via SSH (enabled by default). Try:
   ```bash
   ssh ubuntu@raspberrypi.local
   ```
   or 
   ```bash
   ssh ubuntu@<your-raspberrypi-ip-address>
   ```

7. **Start Using ROS2**:
   ROS2 is pre-installed and configured. You can start using it right away.

## How to Use This Repository

1. **Clone the Repository**
   ```bash
   git clone https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series.git
   cd ros2-getting-started
2. **Follow Along with the Articles**

    This repository complements the "Getting Started with ROS2" article series on Medium.
    Each article will reference specific examples and code from this repository.
    Read the series on [Medium](https://medium.com/@sagarcadet/list/getting-started-with-ros2-adb24ab6d8dd).

## Contributing
Contributions are welcome! If you have any suggestions or improvements, feel free to open an issue or submit a pull request.

## License
This repository is licensed under the Apache 2.0 License. See the [LICENSE](https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series/blob/main/LICENSE) file for more details.

## Contact
For any questions or feedback, feel free to reach out or comment on the Medium articles.
Happy coding!