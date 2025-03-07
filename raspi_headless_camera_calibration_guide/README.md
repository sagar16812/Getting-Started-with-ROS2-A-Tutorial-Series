# Camera Calibration for Raspberry Pi

## Index

1. **[Introduction to Camera Calibration](#introduction-to-camera-calibration)**  
   - Intrinsic Parameters  
   - Obtained Intrinsic Parameters  

2. **[Performing Camera Calibration on Raspberry Pi (Headless Mode)](#performing-camera-calibration-on-raspberry-pi-headless-mode)**  
   - Option 1: Perform Calibration on Your Desktop  
   - Option 2: Capture Images on Raspberry Pi and Calibrate on Desktop  
   - Option 3: Remote GUI via SSH (Alternative Method)  

3. **[Calibration Output Data Description](#calibration-output-data-description)**  
   - Image Resolution  
   - Camera Name  
   - Camera Matrix (Intrinsic Parameters)  
   - Distortion Model  
   - Distortion Coefficients  
   - Rectification Matrix  
   - Projection Matrix  

4. **[FAQs](#faqs)**  
   - How Many Samples Are Required for ROS 2 Camera Calibration?  
   - How to Compare Calibration Results?

## **Introduction to Camera Calibration**
Camera calibration is the process of estimating the **intrinsic parameters** of a camera, which helps in correcting distortions and improving the accuracy of image-based measurements. Intrinsic parameters define the camera’s focal length, optical center, and lens distortion. After calibration, you obtain a calibration file that can be used for **undistorting images** and **accurate vision-based applications**.

### **Obtained Intrinsic Parameters**
After performing the calibration, you will get intrinsic parameters such as:
- **Camera Matrix (K):**
  ```
  K = [ fx  0  cx ]
      [  0  fy  cy ]
      [  0   0   1 ]
  ```
  where:
  - `fx, fy`: Focal lengths in pixels
  - `cx, cy`: Optical center (principal point)
- **Distortion Coefficients:**
  ```
  [k1, k2, p1, p2, k3]
  ```
  These help in correcting lens distortions.

---

## **Performing Camera Calibration on Raspberry Pi (Headless Mode)**
Since the Raspberry Pi is running **Ubuntu Server (without GUI)**, we have multiple ways to perform calibration:

### **Option 1: Perform Calibration on Your Desktop**  

1. **Run the Camera on Raspberry Pi & Stream to Desktop**  
   - Ensure that both devices have ROS 2 installed, are connected to the same network, and have an established ROS 2 communication pipeline. *(If not set up, refer to this [guide](https://github.com/sagar16812/Getting-Started-with-ROS2-A-Tutorial-Series/tree/main/ros2_multi-machine_communication_guide)).*  
   - Start a ROS 2 camera node on the Raspberry Pi to publish the image topic.  
   - On your desktop, subscribe to this topic and use `camera_calibration` to perform the calibration.

2. **Steps:**  
   - On the **Raspberry Pi**, start the ROS 2 camera node:  
     ```bash
     ros2 run image_tools cam2image
     ```
   - On the **Desktop**, view the image stream to verify:  
     ```bash
     ros2 run rqt_image_view rqt_image_view
     ```
   - Run the camera calibration tool on the desktop using a **checkerboard pattern**:  
     ```bash
     ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025
     ```
     *(Modify `--size` and `--square` based on your calibration pattern dimensions.)*  
   - Or, if using a **ChArUco board**, run:  
     ```bash
     ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --pattern charuco --charuco_marker_size 0.02 --aruco_dict 5x5_250
     ```
     *(Modify `--size`, `--square`, `--charuco_marker_size`, and `--aruco_dict` as per your pattern dimensions.)*

3. **Save the Calibration Data**  
   - Once calibrated, the intrinsic parameters will be saved in a `.yaml` file. Use this file in your Raspberry Pi setup.

4. **Verify Calibration**  
   - Run a rectified image stream to check the distortion correction:  
     ```bash
     ros2 run image_proc rectify --ros-args --remap in:=/image_raw --remap out:=/image_rect
     ```
For more queries [jump to FAQs section](#faqs)

---

### **Option 2: Capture Images on Raspberry Pi and Calibrate on desktop**
1. **Manually capture calibration images using `ros2 bag` or a script**:
   - Capture images using a ROS 2 topic and save them.
   - Transfer the images to your desktop.
   - Use OpenCV or ROS 2 `camera_calibration` on the desktop.

2. **Steps:**
   - On **Raspberry Pi**, save images from the camera stream:
     ```bash
     ros2 topic echo /camera/image_raw > image_raw.bag
     ```
   - Transfer the `ros2 bag` file or raw images to your desktop.
   - Use OpenCV’s `calibrateCamera()` or `camera_calibration` package to compute intrinsic parameters.

---

### **Option 3: Remote GUI via SSH (Alternative Method)**
If you prefer a remote GUI session:
```bash
ssh -X ubuntu@raspberrypi.local
```
Then, try running `rqt` tools on your desktop through SSH X11 forwarding.

---

## **Calibration Output Data Description**
After the calibration data is saved, it will save a yaml file at "". The **YAML file** (`ost.yaml`) contains the **intrinsic parameters** of the camera after calibration. Let's break it down:

### **1. Image Resolution**
```
image_width: 320
image_height: 240
```
- The camera resolution used during calibration.
- Future images must have the **same resolution** for the calibration data to be valid.

### **2. Camera Name**
```
camera_name: narrow_stereo
```
- The identifier for the camera, useful when dealing with multiple cameras.

### **3. Camera Matrix (Intrinsic Parameters)**
```
camera_matrix:
  rows: 3
  cols: 3
  data: [303.17322,   0.     , 140.05489,
           0.     , 300.1103 , 113.30547,
           0.     ,   0.     ,   1.     ]
```
This **3×3 matrix** represents the camera's **intrinsic parameters**, which describe:
- **Focal length (fx, fy)**: \( (303.17, 300.11) \)
- **Principal point (cx, cy)**: \( (140.05, 113.31) \) (where the optical axis intersects the image plane)
- The `1` in the last row ensures the **homogeneous coordinates** format.

### **4. Distortion Model**
```
distortion_model: plumb_bob
```
- The distortion model used.  
- **"Plumb Bob"** is a standard **radial & tangential distortion** model.

### **5. Distortion Coefficients**
```
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.141451, -0.090218, -0.014156, -0.022310, 0.000000]
```
- These coefficients correct **lens distortions** (barrel or pincushion effects).
- The five values correspond to:  
  - **Radial distortion** (\( k_1, k_2, k_3 \))  
  - **Tangential distortion** (\( p_1, p_2 \))  

### **6. Rectification Matrix**
```
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0.,
         0., 1., 0.,
         0., 0., 1.]
```
- Used for **stereo cameras**, but for a single camera, it remains an **identity matrix**.
- If stereo calibration was done, this matrix **aligns the left and right images**.

### **7. Projection Matrix**
```
projection_matrix:
  rows: 3
  cols: 4
  data: [314.63394,   0.     , 134.60382,   0.     ,
           0.     , 314.90555, 110.47127,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]
```
- This **3×4 matrix** projects **3D world points onto the image plane**.
- Similar to the **camera matrix**, but includes an **extra column** for 3D-to-2D transformation.
- **Used for rectified images** in stereo vision.

## **FAQs**
### **Q1. How Many Samples Are Required for ROS 2 Camera Calibration?**
The **ROS 2 `camera_calibration` package** automatically collects samples until it determines that **enough diverse images** have been captured for good calibration.

#### **🔹 Minimum Required Samples**
- At least **20-30 good samples** for basic calibration.
- More samples (e.g., **50-100**) improve accuracy, especially for **lens distortion correction**.
- The process **stops automatically** once it gathers enough **diverse** samples.

#### **🔹 When Does Calibration Complete?**
- The tool does **not stop at a fixed number** but ensures:
  - The **chessboard/ChArUco board is detected in different positions and angles**.
  - The images cover **various parts of the camera’s field of view**.
  - It gets **good corner detections**.

- **You will see a message** when it has enough data:
  ```
  Calibration complete! Press [CALIBRATE] to compute, [SAVE] to store results.
  ```
  Then, press **CALIBRATE** to compute the parameters.

### **Q2. How to Compare Calibration Results?**
To analyse and compare different calibration methods (e.g., **Checkerboard vs ChArUco board calibration**):

#### **1. Compare Intrinsic Parameters**
- Print and compare:
  - **Camera Matrix (fx, fy, cx, cy)**
  - **Distortion Coefficients (k1, k2, k3, p1, p2)**

#### **2. Reprojection Error Analysis**
- The reprojection error measures how well the estimated parameters fit the actual detected points.
- Lower error = **better calibration**.

#### **3. Visualise Undistorted Images**
- Apply undistortion using:
  ```bash
  cv2.undistort(image, camera_matrix, dist_coeffs)
  ```
- Compare images from different calibration results.

#### **4. Projected Points Alignment**
- Overlay detected corners vs projected points.
- If projected points misalign significantly, one method might be inaccurate.

---

### **Q3. How to Improve Calibration Accuracy?**
1. **Move the board around**:
   - Close & far, tilted at different angles.
   - Cover the entire field of view (not just the center).
2. **Ensure good lighting** (avoid glare & shadows).
3. **Wait for sample count to stabilise** before pressing `CALIBRATE`.

**Once you have the calibration results, save them and use them in your ROS 2 camera pipeline!**

