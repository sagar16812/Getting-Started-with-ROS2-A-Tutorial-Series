# **ROS 2 Bag Recording and Playback Guide**  

ROS 2 provides a **rosbag** feature to record and replay data from topics. This is useful for debugging, simulation, and data analysis.  

## **1️. Recording a ROS 2 Bag File**  
To record specific topics:  
```bash
ros2 bag record /topic_name1 /topic_name2
```  
Example: Record an image and IMU topic:  
```bash
ros2 bag record /camera/image_raw /imu/data
```  
To record **all topics**, use:  
```bash
ros2 bag record -a
```

### **Save Bag Files to a Custom Directory**  
```bash
ros2 bag record -o my_bag /topic_name
```
This will save the recorded bag file in a folder named `my_bag`.  

---

## **2️. Playing Back a ROS 2 Bag File**  
To replay a recorded bag file:  
```bash
ros2 bag play my_bag
```
This will republish the recorded topics with the original timestamps.  

### **Change Playback Rate**  
- **Speed up playback (e.g., 2x faster)**  
  ```bash
  ros2 bag play my_bag -r 2.0
  ```
- **Slow down playback (e.g., 0.5x slower)**  
  ```bash
  ros2 bag play my_bag -r 0.5
  ```

### **Loop Playback**  
To continuously replay the bag file in a loop:  
```bash
ros2 bag play my_bag --loop
```

---

## **3️. Checking Bag File Contents**  
To list recorded topics inside a bag file:  
```bash
ros2 bag info my_bag
```

---

## **4️. Extract Data from a ROS 2 Bag File**  
You can extract messages from a recorded bag file using:  
```bash
ros2 bag reindex my_bag
```
Or convert the data to a readable format (e.g., CSV or JSON) using Python.  

---

## **5️. Troubleshooting**  
If playback doesn't work:  
- Ensure the correct `ROS_DOMAIN_ID` is set.  
- Make sure the topics being played match the expected subscribers.  
