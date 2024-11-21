
###  `README.md` for `camera_test_pkg` Package

```markdown
# camera_test_pkg

## Overview

The `camera_test_pkg` package is a ROS 2 package designed to integrate and test camera feeds for the Panther robot using the ROS 2 framework. It provides functionality for object detection and tracking using the YOLO model and publishes the robot's pose based on camera data. This package is useful for scenarios requiring visual-based navigation and monitoring.

### Features

- **Object Detection**: Utilizes the YOLO model for detecting objects such as `person`, `panther_robot`, `car`, and more.
- **Camera Integration**: Supports multiple camera views, including top-down and side views.
- **Dynamic Camera Switching**: Automatically switches between cameras if no object is detected for a specified duration.
- **Pose Estimation**: Publishes the robot's pose based on detection results using camera data.
- **Real-time Visualization**: Provides real-time image feed with grid overlays for debugging and visual analysis.

## Prerequisites

- ROS 2 (Foxy, Galactic, or Humble)
- Python packages: `numpy`, `opencv-python`, `cv_bridge`, `tf_transformations`, `ultralytics` (for YOLO), `matplotlib`
- YOLO Model: Download or train a YOLO model and place it in the appropriate directory.

## Installation

1. **Clone the Repository:**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your_username/camera_test_pkg.git
    ```

2. **Install Dependencies:**
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **Build the Package:**
    ```bash
    colcon build --packages-select camera_test_pkg
    ```

4. **Source the Setup Script:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

### Launching the Integrated Camera Node

To run the integrated camera node for object detection and pose estimation:

```bash
ros2 run camera_test_pkg sensor_fusion_covariance.py
