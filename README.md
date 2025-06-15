# 🛠️ Oil Pan Tracking with Azure Kinect and xArm 

This project implements a visual servoing pipeline that enables an xArm robotic arm to detect, track, and interact with an oil pan using RGB-D data from an Azure Kinect sensor.

The system includes:
- Color-based segmentation
- 3D point cloud pose estimation using RANSAC + ICP
- Real-time robot control based on the estimated pose

![Final](https://github.com/user-attachments/assets/b20255a8-7cdc-44ef-b2ec-157498bf71b1)

## 🔧 Requirements

- ROS 2 Humble (recommended)
- Python 3.8+
- Azure Kinect DK + [azure_kinect_ros2_driver](https://github.com/ckennedy2050/Azure_Kinect_ROS2_Driver)
- UFactory xArm 6 + [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)
- Libraries: Open3D, NumPy, OpenCV
