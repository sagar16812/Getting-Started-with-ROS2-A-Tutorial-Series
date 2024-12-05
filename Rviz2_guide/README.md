# README: Visualising Sample Datasets with RViz2  

This guide demonstrates how to visualise sample datasets from NVIDIA's R2B Dataset (2023) using **RViz2**, a powerful visualisation tool in ROS2. We will focus on visualising topics related to images and LIDAR data from the **r2b_lounge** and **r2b_hallway** sequences.

---

## Contents  
1. [Introduction](#introduction)  
2. [Prerequisites](#prerequisites)  
3. [Installing RViz2](#installing-rviz2)  
4. [Downloading and Organising the Dataset](#downloading-and-organising-the-dataset)  
5. [Visualising the Dataset in RViz2](#visualising-the-dataset-in-rviz2)  
6. [Troubleshooting](#troubleshooting)  

---

## Introduction  
RViz2 is a visualisation tool used to display sensor data such as images, point clouds, and transforms in ROS2 systems. It is essential for debugging and understanding robotics data pipelines.

In this guide, we use RViz2 to visualise sample datasets from the **R2B Dataset 2023**, focusing on LIDAR and camera topics.

---

## Prerequisites  
- **ROS 2 Humble** installed on your system.  
- Basic familiarity with ROS 2 commands and concepts.  

---

## Installing RViz2  
Install RViz2 using the following command:  
```bash  
sudo apt install ros-humble-rviz2  
```  

---

## Downloading and Organising the Dataset  
1. Visit the [NVIDIA R2B Dataset 2023 Website](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2023).  
2. Navigate to the **File Browser** tab and download the `.yaml` and `.db3` files for:  
   - **r2b_lounge**  
   - **r2b_hallway**  

3. Create a directory structure to store the datasets:  
   ```bash  
   mkdir -p ~/ros2Datasets/{r2b_lounge,r2b_hallway}  
   ```  
4. Move the downloaded files to their respective directories.

---

## Visualising the Dataset in RViz2  

### Setup  
1. Open three terminal windows:  
   - **Terminal 1**: Launch RViz2:  
     ```bash  
     rviz2  
     ```  
     Minimise for now.  

   - **Terminal 2**: Navigate to the dataset directory and play the bag file:  
     ```bash  
     ros2 bag play <path-to-db3-file> -r 0.1  
     ```  
     Pause playback using the **space bar** and reduce speed using the **down arrow key** to 0.1.  

   - **Terminal 3**: Explore available topics:  
     ```bash  
     ros2 topic list  
     ```  
     Inspect a specific topic (e.g., LIDAR):  
     ```bash  
     ros2 topic echo /pandar_xt_32_0_lidar  
     ```  
     Resume and pause playback in **Terminal 2** to observe changes in the **Terminal 3**, explore the frame id, It would be like frame_id: PandarXT-32_0, note it down. Stop echoing with `Ctrl+C`.

### Visualising in RViz2  
1. Add visualisation elements in RViz2:  
   - **PointCloud2** for `/pandar_xt_32_0_lidar`  
   - **Image** for `/d455_1_rgb_image` and `/d455_1_depth_image`.  
2. Set **Global Option -> Fixed Frame** to `PandarXT-32_0`.  
3. Resume playback in **Terminal 2** to view live data in RViz2.  

### Save and Reuse the Configuration
After setting up your displays:
1. Save the RViz2 configuration:
   ```bash
   File > Save Config As...
   ```
2. Reload the configuration:
   ```bash
   rviz2 -d <path-to-config>
   ```
---

## Troubleshooting  
- **Topic not visible in RViz2**:  
  - Check topic availability with `ros2 topic list`.  
  - Verify bag playback using `ros2 bag info`.  

- **RViz2 not displaying data**:  
  - Ensure the **Fixed Frame** is correctly set in RViz2.

---

## Conclusion  
By following this guide, you learned how to visualise topics from R2B datasets using RViz2. This process helps in debugging and gaining insights into sensor data, such as LIDAR point clouds and camera images, for robotics applications.

---

For further queries or contributions, feel free to raise issues or pull requests!  
