# Camera Calibration for Raspberry Pi

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
Since the Raspberry Pi 3 is running **Ubuntu Server (without GUI)**, we have multiple ways to perform calibration:

### **Option 1: Perform Calibration on Your Laptop**
1. **Run the Camera on Raspberry Pi & Send the Stream to Laptop**  
   - Start a ROS 2 camera node on the Raspberry Pi to publish the image topic.
   - On your laptop, subscribe to this topic and use `camera_calibration` to perform calibration.

2. **Steps:**
   - On the **Raspberry Pi**, start a ROS 2 camera node:
     ```bash
     ros2 run image_tools cam2image
     ```
   - On the **laptop**, view the image stream to verify:
     ```bash
     ros2 run rqt_image_view rqt_image_view
     ```
   - Run the camera calibration tool on the laptop:
     ```bash
     ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --camera_namespace /camera
     ```
     *(Modify `--size` and `--square` based on your calibration pattern dimensions)*

3. **Save the Calibration Data**  
   - Once calibrated, store the `yaml` file and use it in your Raspberry Pi setup.

---

### **Option 2: Capture Images on Raspberry Pi and Calibrate on Laptop**
1. **Manually capture calibration images using `ros2 bag` or a script**:
   - Capture images using a ROS 2 topic and save them.
   - Transfer the images to your laptop.
   - Use OpenCV or ROS 2 `camera_calibration` on the laptop.

2. **Steps:**
   - On **Raspberry Pi**, save images from the camera stream:
     ```bash
     ros2 topic echo /camera/image_raw > image_raw.bag
     ```
   - Transfer the `ros2 bag` file or raw images to your laptop.
   - Use OpenCV’s `calibrateCamera()` or `camera_calibration` package to compute intrinsic parameters.

---

### **Option 3: Remote GUI via SSH (Alternative Method)**
If you prefer a remote GUI session:
```bash
ssh -X ubuntu@raspberrypi.local
```
Then, try running `rqt` tools on your laptop through SSH X11 forwarding.

---

## ** How Many Samples Are Required for ROS 2 Camera Calibration?**
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

---

## ** How to Compare Calibration Results?**
To analyse and compare different calibration methods (e.g., **Checkerboard vs ChArUco board calibration**):

### **1. Compare Intrinsic Parameters**
- Print and compare:
  - **Camera Matrix (fx, fy, cx, cy)**
  - **Distortion Coefficients (k1, k2, k3, p1, p2)**

### **2. Reprojection Error Analysis**
- The reprojection error measures how well the estimated parameters fit the actual detected points.
- Lower error = **better calibration**.

### **3. Visualise Undistorted Images**
- Apply undistortion using:
  ```bash
  cv2.undistort(image, camera_matrix, dist_coeffs)
  ```
- Compare images from different calibration results.

### **4. Projected Points Alignment**
- Overlay detected corners vs projected points.
- If projected points misalign significantly, one method might be inaccurate.

---

## ** How to Improve Calibration Accuracy?**
1. **Move the board around**:
   - Close & far, tilted at different angles.
   - Cover the entire field of view (not just the center).
2. **Ensure good lighting** (avoid glare & shadows).
3. **Wait for sample count to stabilise** before pressing `CALIBRATE`.

🚀 **Once you have the calibration results, save them and use them in your ROS 2 camera pipeline!** 🎯

