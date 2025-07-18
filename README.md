2D-to-3D LiDAR Slicer with ROS 2

This project demonstrates 3D perception using a 2D LiDAR. It contains a ROS 2 package that converts LaserScan data from an RPLIDAR A1 into a 3D PointCloud2 message based on a manually controlled tilt angle, serving as a foundation for more complex 3D mapping and SLAM systems.


Key Features

    Real-Time 2D-to-3D Conversion: Converts LaserScan messages to PointCloud2 messages on the fly.

    Manual Tilt Control: Uses a ROS 2 parameter to dynamically set the LiDAR's vertical tilt angle, allowing you to "paint" a 3D scene.

    Lightweight & Modular: Built with minimal dependencies for easy integration into larger projects.

    ROS 2 Humble: Developed and tested on ROS 2 Humble Hawksbill.

Getting Started

These instructions will get the project running on your local machine.
Prerequisites

    Ubuntu 22.04 with ROS 2 Humble installed.

    An RPLIDAR A1 or similar 2D LiDAR.

    Python 3.8+

Installation

    Clone the repository into a new ROS 2 workspace:

    mkdir -p ~/ros2_lidar_ws/src
    cd ~/ros2_lidar_ws/src
    git clone https://github.com/Vanshh2111/ros2-lidar-mapper.git . 

    Install Dependencies:
    Navigate to the workspace root and install the necessary ROS 2 packages.

    cd ~/ros2_lidar_ws/
    sudo apt update
    sudo apt install ros-humble-rplidar-ros -y

    Build the Workspace:

    colcon build

Usage

Follow these steps to run the LiDAR slicer.

    Plug in your RPLIDAR to a USB port on your computer.

    Grant USB Port Permission:

    sudo chmod 666 /dev/ttyUSB0

    Source the Workspace and Run:
    In a new terminal, source the setup file and run the launch file.

    source ~/ros2_lidar_ws/install/setup.bash
    ros2 launch pointcloud_slicer slicer.launch.py

Visualization and Control

    Open RViz2:
    In a new terminal, run rviz2 after sourcing ROS 2 (source /opt/ros/humble/setup.bash).

        Set the Fixed Frame to base_link.

        Click "Add" -> "By topic" and select the /point_cloud_slice topic.

    Control the Tilt Angle:
    In another new terminal, source your workspace and use the ros2 param set command to change the angle. The slice will move up and down in RViz in real-time.

    # Example for 20 degrees up
    ros2 param set /manual_slicer_node tilt_angle_degrees 20.0

    # Example for 15 degrees down
    ros2 param set /manual_slicer_node tilt_angle_degrees -15.0
