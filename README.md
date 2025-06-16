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

## 📁 Reference Model: Oil Pan
You must include a reference .ply model of the object to perform registration.
- Store your file in the oil_pan/ directory
- Example: oil_pan_top.ply
- You can generate .ply models using photogrammetry tools or download scanned versions

🚀 How to Launch the System

1. Launch the Azure Kinect sensor:
ros2 run azure_kinect_ros2_driver azure_kinect_node

2. Connect to the xArm
Replace the IP with your arm's actual IP address (e.g., 192.168.1.208):
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.208 [add_gripper:=true]

3. Run the Main Pipeline (Segmentation + Pose Estimation + Robot Control)
ros2 launch xarm_vision segmentation_point_cloud.launch.py

4. (Optional) Launch RViz for Visualization


