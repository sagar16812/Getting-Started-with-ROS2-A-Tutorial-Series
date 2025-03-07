# Getting Started with ROS2

---

## Index

1. **[Introduction](#introduction)** : Overview of the repository and Medium article series  
2. **[How to Use This Repository](#how-to-use-this-repository)** : Steps to follow for theoretical and hands-on learning  
3. **[Article List](#article-list)** : Indexed list of articles with corresponding ROS2 example packages  
4. **[Guides and Documentation](#guides-and-documentation)** : Installation, multi-machine communication, camera calibration, and ROS2 bag recording guides  
5. **[Example Packages](#list-of-example-packages)** : Brief descriptions of included example packages  
6. **[Raspberry Pi Image for ROS2](#raspberry-pi-image-with-ubuntu-server-2204-and-ros2-humble-base--perception)** : Pre-configured image details and setup instructions  
7. **[Contributing](#contributing)** : Guidelines for contributions  
8. **[License](#license)** : Repository licensing information  
9. **[Contact](#contact)** : How to reach out for queries or feedback

---

## Introduction
This repository complements the **"Getting Started with ROS2"** article series on Medium and it contains multiple guides and example packages for working with ROS 2. Follow the [article series on Medium](https://medium.com/@sagarcadet/list/getting-started-with-ros2-adb24ab6d8dd).

You can also check out the pre-configured Raspberry Pi image with **[Ubuntu Server 22.04 and ROS 2 Humble (Base + Perception)](#raspberry-pi-image-with-ubuntu-server-2204-and-ros2-humble-base--perception)**.

---

## How to Use This Repository

1. Read the articles for theoretical understanding.
2. Clone this repository and navigate to the relevant example package.
3. Follow the guides for hands-on implementation.

---

## Article List
|**Sr.no.** | **Article** | **Example Package** |
|---------|---------|----------------|
| 1. | [An Introduction](https://medium.com/spinor/getting-started-with-ros2-an-introduction-a36e21ff5feb) | <center>---</center> |
| 2. | [Why ROS2?](https://medium.com/spinor/getting-started-with-ros2-why-ros2-f4980d63e9fc) | <center>---</center> |
| 3. | [Install and Setup ROS2 Humble on Ubuntu 22.04(LTS)](https://medium.com/spinor/getting-started-with-ros2-install-and-setup-ros2-humble-on-ubuntu-22-04-lts-ad718d4a3ac2) | <center>---</center> |
| 4. | [Overview of ROS2 Workspaces, Packages and Nodes](https://medium.com/spinor/getting-started-with-ros2-overview-of-ros2-workspaces-packages-and-nodes-dac0042d7d48) | <center>---</center> |
| 5. | [Create and Set Up a Workspace](https://medium.com/spinor/getting-started-with-ros2-create-and-set-up-a-workspace-f60a6c52328c) | <center>---</center> |
| 6. | [Understanding Nodes](https://medium.com/spinor/getting-started-with-ros2-understanding-nodes-075e9cde2863) | <center>---</center> |
| 7. | [How to Create Custom Packages and Nodes (with Python)](https://medium.com/spinor/getting-started-with-ros2-how-to-create-custom-packages-and-nodes-with-python-6b152c7b4e76) | [simple_node_example](workspace/ros2_ws/src/simple_node_example/) |
| 8. | [Understanding ROS2 Topics and Messages](https://medium.com/spinor/getting-started-with-ros2-understanding-ros2-topics-and-messages-78fe9aa4c9db) | [pub_sub_tutorial](workspace/ros2_ws/src/pub_sub_tutorial/) |
| 9. | [What is a ROS2 Service? Implementing ROS2 Service-Client Nodes in Python](https://medium.com/spinor/getting-started-with-ros2-what-is-a-ros2-service-implementing-ros2-service-client-nodes-in-python-e59456a1bd92) | [battery_status](workspace/ros2_ws/src/battery_status/) |
| 10. | [Asynchronous Task Handling using ROS2 Actions](https://medium.com/spinor/getting-started-with-ros2-asynchronous-task-handling-using-ros2-actions-0edef14e6be4) | <center>---</center> |
| 11. | [Implementing Custom Actions in ROS2](https://medium.com/spinor/getting-started-with-ros2-implementing-custom-actions-in-ros2-438ac1fb929f) | [rotate_turtle_action_client](workspace/ros2_ws/src/rotate_turtle_action_client/) |
| 12. | [A Step-by-Step Guide to Setting Up a Camera for ROS2 on Raspberry Pi and Desktop](https://medium.com/spinor/getting-started-with-ros2-a-step-by-step-guide-to-setting-up-a-camera-for-ros2-on-raspberry-pi-and-6b8fe8cbb9df) | <center>---</center> |
| 13. | [Mastering Data Recording and Playback with ROS2 Bag](https://medium.com/spinor/getting-started-with-ros2-mastering-data-recording-and-playback-with-ros2-bag-68cdc1de6725) | <center>---</center> |

---

## Guides and Documentation

- **installation_guide/**: Step-by-step instructions for installing ROS 2 on a desktop, laptop, or Raspberry Pi, along with environment setup guidelines.
- **ros2_multi-machine_communication_guide/**: A comprehensive guide on configuring ROS 2 for multi-machine communication.
- **raspi_headless_camera_calibration_guide/**: Instructions for performing camera calibration on a Raspberry Pi without a graphical user interface.
- **ros2_bag_recording_and_playback_guide/**: Guidelines on recording and replaying ROS 2 bag files for data analysis and debugging.
- **Rviz2_guide/**: A guide to using Rviz2 for visualizing ROS 2 data from active nodes or recorded rosbag files.

---

## List of example Packages:

- **simple_node_example**: A basic example demonstrating a simple ROS 2 node.
- **pub_sub_tutorial**: Illustrates the implementation of publisher and subscriber nodes for ROS 2 topics.
- **battery_status**: Demonstrates client-server node communication in ROS 2.
- **rotate_turtle_action_client**: Implements an action client node to control the movement of the Turtlesim TurtleBot.

---

## Raspberry Pi Image with Ubuntu Server 22.04 and ROS2 Humble Base + Perception

I have created a Raspberry Pi image with **Ubuntu Server 22.04 for arm64 and ROS2 base + perception** pre-installed to help you get started quickly. This image can be downloaded directly from this [link](https://drive.google.com/file/d/1yvv7u4Z7PgbNEOfC1JUykrVPpexAQosP/view?usp=sharing), flash it to a SD card, and boot on a Raspberry Pi 3 or above model. In case, you are not able to use this image, you can install ROS2 on Raspberry Pi by checking the `installation_guide` directory or following this [link](https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series/tree/main/installation_guide).

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

---

## Contributing
Contributions are welcome! If you have any suggestions or improvements, feel free to open an issue or submit a pull request.

## License
This repository is licensed under the Apache 2.0 License. See the [LICENSE](https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series/blob/main/LICENSE) file for more details.

## Contact
For any questions or feedback, feel free to reach out or comment on the Medium articles.
Happy coding!