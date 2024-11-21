# test_custom_panther

## Overview

The `test_custom_panther` package is a ROS 2 package designed to simulate and control the Panther robot using Gazebo and ROS 2. This package includes launch files and configuration settings to support various robot simulation scenarios, including different wheel types and sensor setups. It provides an integration with Ignition Gazebo for realistic simulation and offers support for camera-based navigation and pose estimation.

### Features

- **Multiple Wheel Configurations**: Supports different wheel types (e.g., `WH01`, `WH02`, `WH04`, `CUSTOM`) for versatile simulation scenarios.
- **Launch Scripts**: Includes launch files for setting up robot states, initializing simulation environments, and integrating camera views.
- **Gazebo Integration**: Provides seamless integration with Gazebo using the Ignition Gazebo plugin.
- **Transforms and TF**: Publishes transforms between various frames (world, map, odom, base_link) for accurate localization.

## Prerequisites

- ROS 2 (Foxy, Galactic, or Humble)
- Gazebo with Ignition support
- ROS 2 Gazebo plugins (`ros_gz_sim`, `ros_gz_bridge`)
- Additional ROS 2 packages: `panther_description`, `panther_controller`, `panther_gazebo`

## Installation

1. **Clone the Repository:**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your_username/test_custom_panther.git
    ```

2. **Install Dependencies:**
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **Build the Package:**
    ```bash
    colcon build --packages-select test_custom_panther
    ```

4. **Source the Setup Script:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

### Launching the Simulation

To start the simulation with the default configuration:

```bash
ros2 launch test_custom_panther test_custom_panther.launch.py'''


