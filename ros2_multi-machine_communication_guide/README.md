# ROS2 Multi-Machine Communication Guide

## Connecting Your Raspberry Pi's ROS2 System with Your Desktop's ROS2 System

This guide will help you set up a ROS2 communication pipeline and connect your Raspberry Pi's ROS2 system with your desktop's ROS2 system, allowing your desktop to subscribe to any topic from the Raspberry Pi and vice-versa. Follow the steps below to set up ROS2 communication between the two devices.

## Steps to Connect the Systems

### 1. Ensure Network Connectivity
Make sure both your Raspberry Pi and desktop are on the same network and can ping each other.

### 2. Set Up `ROS_DOMAIN_ID`
To ensure communication between the devices, you need to set the same `ROS_DOMAIN_ID` on both the Raspberry Pi and desktop:

- Open the `~/.bashrc` file on both devices and add the following line:

    ```bash
    export ROS_DOMAIN_ID=<choose_a_number_between_0_and_232>
    ```

- After adding the line, run the following command or restart the terminal to apply the changes:

    ```bash
    source ~/.bashrc
    ```

### 3. Configure `ROS_LOCALHOST_ONLY`
Make sure `ROS_LOCALHOST_ONLY` is not set to `1`, as it will restrict communication to localhost. To ensure it's unset, add the following line to the `~/.bashrc` file on both devices:

```bash
unset ROS_LOCALHOST_ONLY
```

Then, source the `.bashrc` file again:

```bash
source ~/.bashrc
```

### 4. Set Up Multicast Routes (If Needed)
If you're facing issues with discovery, you may need to add multicast routes. Run the following command on both devices:

```bash
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev <network_interface>
```

Replace `<network_interface>` with the appropriate network interface name (e.g., `wlan0` for WiFi or `eth0` for Ethernet).

---
### 5. Running the Camera Node:
Open a terminal on your Raspberry Pi or ssh into it, then run the ROS2 camera node with the following command:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]"
```

### 6. Verify and Get Image Data:
Open a terminal on your Desktop Computer, then use the following commands to check if the camera is publishing data:

```bash
ros2 topic list
```

Make sure `/image_raw` is listed. If it appears, you can view the image data by running:

```bash
ros2 topic echo /image_raw
```

### 7. View the Image:
The above only gives you the image data but you won't be able to see any image preview. For that you need `rqt_image_view` package.

First make sure you have installed the required packages on your Desktop:
```bash
sudo apt install ros-humble-image-transport-plugins ros-humble-image-pipeline
sudo apt install ros-humble-rqt-image-view
```

Assuming that you have started the camera node on Raspberry Pi, start the camera preview using `rqt_image_view`:
```bash
ros2 run rqt_image_view rqt_image_view
```

A window will appear, from the first dropdown menu, select the image topic. You can select either `/image_raw` or `/image_raw/compressed`.

---

By following these steps, your Raspberry Pi and desktop should be able to communicate through ROS2, allowing your desktop to subscribe to topics such as `/image_raw` from the Raspberry Pi.

