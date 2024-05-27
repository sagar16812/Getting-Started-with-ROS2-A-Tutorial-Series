# Install ROS2 Humble on Raspberry Pi without Monitor

This guide provides a step-by-step process for installing ROS2 Humble on a Raspberry Pi running Ubuntu Server 22.04 LTS, all without the need for a monitor. For instructions on installing ROS2 on a desktop PC, refer to this [guide](https://medium.com/spinor/getting-started-with-ros2-install-and-setup-ros2-humble-on-ubuntu-22-04-lts-ad718d4a3ac2).

## Steps

1. **Download the Image**:
   - Download the [Ubuntu Server 22.04 LTS](https://ubuntu.com/download/raspberry-pi) image. Choose the 64-bit version.
   - Alternatively, you can select the image directly in the Raspberry Pi Imager in the next step.

2. **Install the Imager**:
   - Install the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) appropriate for your operating system.

3. **Burn the Image to SD Card**:
   - Follow the official installation documentation [here](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview).
   - Write the Ubuntu Server 22.04 LTS image to an SD card (at least 32GB). Ensure you set the hostname, enable SSH, set the username and password, configure Wi-Fi SSID, and other required configurations before writing to the SD card.

4. **Boot the Raspberry Pi**:
   - Insert the SD card into the Raspberry Pi and power it on.
   - Wait at least 5 minutes for the first boot to complete.

5. **Get Your Raspberry Pi's IP Address**:
   - Ensure your system and Raspberry Pi are connected to the same Wi-Fi network.
   - Ping the Raspberry Pi using its hostname. For example, if your hostname is `raspberrypi`, use:
     ```bash
     ping raspberrypi.local
     ```
   - Note the IP address of the Raspberry Pi.

6. **SSH into the Raspberry Pi**:
   - If the Raspberry Pi's username is `ubuntu` and the IP address is `192.168.1.23`, connect via SSH:
     ```bash
     ssh ubuntu@raspberrypi.local
     ```
     or
     ```bash
     ssh ubuntu@192.168.1.23
     ```
   - If you encounter an error due to a previous SSH key, run:
     ```bash
     ssh-keygen -f ~/.ssh/known_hosts -R <raspberrypi-ip-address>
     ```

7. **Install ROS2 Humble on the Raspberry Pi**:
   - **Set Locale**:
     ```bash
     locale  # check for UTF-8
     sudo apt update && sudo apt install locales
     sudo locale-gen en_US en_US.UTF-8
     sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
     export LANG=en_US.UTF-8
     locale  # verify settings
     ```
   - **Setup Sources**:
     ```bash
     sudo apt install software-properties-common
     sudo add-apt-repository universe
     sudo apt update && sudo apt install curl -y
     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
     ```
   - **Install ROS2 Packages**:
     ```bash
     sudo apt update
     sudo apt upgrade
     sudo apt install ros-humble-ros-base
     sudo apt install ros-dev-tools
     sudo apt-get install ros-humble-demo-nodes-cpp -y
     sudo apt-get install ros-humble-demo-nodes-py -y
     sudo apt install ros-humble-perception
     ```
   - **Environment Setup**:
     ```bash
     source /opt/ros/humble/setup.bash
     ```
     (Replace `.bash` with your shell if you're not using bash, e.g., `.sh` or `.zsh`)

8. **Run Examples**:
   - **Talker-Listener Example**:
     - In one terminal/SSH session:
       ```bash
       source /opt/ros/humble/setup.bash
       ros2 run demo_nodes_cpp talker
       ```
     - In another terminal/SSH session:
       ```bash
       source /opt/ros/humble/setup.bash
       ros2 run demo_nodes_py listener
       ```

9. **Bonus Step**:
   - To automate the environment setup process, add the command to source the setup file in the `.bashrc` file:
     ```bash
     echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
     source ~/.bashrc
     ```

For further details, you can follow the [official ROS2 installation guide for Raspberry Pi](https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html).