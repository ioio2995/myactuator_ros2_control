# MyActuator ROS 2 Control

This project provides a ROS 2 driver for the MyActuator RMD X-series servo motors, utilizing the SDK from [2b-t/myactuator_rmd](https://github.com/2b-t/myactuator_rmd).

## Project Structure

- **myactuator_demo_bringup**: Launch files for ROS 2 nodes.
- **myactuator_demo_description**: URDF files for robot description.
- **myactuator_hardware_interface**: Hardware interface implementation.

## Prerequisites

- ROS 2 (Humble Hawksbill or later)
- Python 3.6+
- Build tools (CMake)
- MyActuator RMD SDK dependencies:
  ```bash
  sudo apt-get install -y build-essential cmake can-utils iproute2 linux-modules-extra-$(uname -r)
  sudo apt-get install -y python3 python3-pip python3-pybind11
  ```

## Installation

1. Create a ROS 2 workspace and navigate into it:
   ```bash
   mkdir -p ~/ros2_ws/src/myactuator
   cd ~/ros2_ws/src/myactuator
   ```

2. Clone the repository into the workspace:
   ```bash
   git clone https://github.com/ioio2995/myactuator_ros2_control.git
   git clone https://github.com/2b-t/myactuator_rmd.git
   cd ../..
   ```

3. Install ROS 2 dependencies:
   ```bash
   sudo apt update
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Install Python dependencies:
   ```bash
   cd src/myactuator_ros2_control
   pip install -r requirements.txt
   cd ../..
   ```

5. Build the workspace:
   ```bash
   colcon build
   ```

6. Source the setup script:
   ```bash
   source install/setup.bash
   ```

## Usage

To launch the system:
```bash
ros2 launch myactuator_demo_bringup bringup.launch.py
```

## License and Attribution

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contribution

Contributions are welcome. Please submit pull requests and open issues for suggestions or bugs.