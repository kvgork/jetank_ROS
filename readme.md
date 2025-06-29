# [DEPRECATED] JeTank ROS Integration

UPDATE[29/06/2025]: Got my hand on a orin nano super developer kit, so will be fitting that in the frame at start over on ROS2! 

<div align="center">
  <img src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/j/e/jetank-ai-kit-1.jpg" alt="JeTank Robot with ROS Integration" />
  <p><i>ROS integration for the Waveshare JeTank AI Kit with autonomous sock collection functionality</i></p>
</div>

## 🤖 Project Overview

This repository provides a complete Robot Operating System (ROS) integration for the [Waveshare JeTank AI Kit](https://www.waveshare.com/wiki/JETANK_AI_Kit), transforming it into a fully autonomous robot platform. Building upon the [original Waveshare JeTank repository](https://github.com/waveshare/JETANK), this project adds ROS compatibility and extends the robot's capabilities to include autonomous sock detection and collection in indoor environments.

I hope to upgrade this project to ROS2 as soon as I get my hands on a Jetson Orin Super Development kit. Then I also want to try to create functions using Reinforcement Learning.

This is all work in progress!

## 🛠️ Hardware updates

- **Camera module**: Updated the stock IMX219-160 Camera module to a IMX219-83 Stereo Camera module to allow for depth perception

## ✨ Key Features

- **Complete ROS Integration**: Full conversion of the JeTank platform to ROS architecture
- **ROS Control Package**: Comprehensive drivers for all JeTank hardware:
  - Motor control interface
  - Camera integration
  - Servo controllers
  - Sensor data publishing
- **Autonomous Navigation**: SLAM-based mapping and path planning
- **Computer Vision Pipeline**: Object detection system for sock identification
- **Custom Gripper Mechanism**: Design files for sock collection attachment

## 🛠️ Technologies & Skills Demonstrated

- **ROS Melodic**: Full node architecture with publishers/subscribers
- **NVIDIA Jetson Nano**: Optimized for the B01 Developer Kit
- **Python**: Clean, modular codebase for hardware interfacing
- **C++**: Performance-critical components
- **Computer Vision**: OpenCV and TensorFlow Lite integration
- **SLAM**: Implementation for autonomous navigation
- **Hardware Integration**: Bridging Jetson Nano with ROS ecosystem

## 🏗️ System Architecture

To be refined.
```
├── jetank_driver/
│   ├── scripts/        # Python driver scripts for hardware control
│   ├── launch/         # Launch files for the robot
│   └── src/            # C++ source code for performance-critical components
│
├── jetank_description/
│   ├── urdf/           # Robot model description files
│   ├── meshes/         # 3D models for visualization
│   └── launch/         # URDF launch files
│
├── jetank_navigation/
│   ├── config/         # Navigation stack configuration
│   ├── launch/         # Launch files for autonomous navigation
│   └── maps/           # Saved environment maps
│
└── jetank_sock_collector/
    ├── perception/     # Computer vision for sock detection
    ├── planning/       # Task planning for collection routine
    └── manipulation/   # Gripper control for sock collection
```

## 📋 Project Roadmap & TODO

- [x] Set up ROS package structure
- [x] Update hardware
- [x] Complete URDF model for visualization in RViz

Drive control
- [x] Implement basic motor control through ROS

Arm control
- [x] Develop servo control interface
- [x] Design and implement gripper attachment
- [ ] Create MoveIt! config for the arm and gripper

Camera implementation
- [x] Create camera node for video streaming
- [x] Create camera calibration script
- [x] Create camera node for point could generation
- [ ] Make compressed image for point cloud generation
- [ ] Optimize point cloud generation

Vision implementation
- [ ] Add item segemntation model
- [ ] Create sock detection algorithm

Autonomy implementation
- [ ] Implement odometry for tracking robot movement(Might not be possible due to lack of sensors)
- [ ] Integrate SLAM for autonomous navigation
- [ ] Develop autonomous sock collection behavior
- [ ] Test and optimize in various environments

Finishing up
- [ ] Create comprehensive documentation and tutorials
- [ ] Split package into seperate package based on function

## 🚀 Installation & Setup

To be defined

<!-- ```bash
# Install ROS Melodic (on Ubuntu 18.04, compatible with Jetson Nano B01)
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Initialize and update rosdep
sudo rosdep init
rosdep update

# Create and build catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# Add ROS environment to bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Clone this repository
cd ~/catkin_ws/src/
git clone https://github.com/kvgork/jetank_ROS.git
cd ~/catkin_ws/
catkin_make

# Install additional dependencies
sudo apt install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-gmapping ros-melodic-navigation

# Setup JeTank hardware interface
cd ~/catkin_ws/src/jetank_ROS/setup
sudo ./setup_hardware.sh
``` -->

## 📁 Usage

To be defined
<!-- 
```bash
# Launch the basic JeTank ROS driver
roslaunch jetank_driver jetank_base.launch

# Launch with camera and visualization
roslaunch jetank_driver jetank_full.launch

# Launch teleop control with keyboard
roslaunch jetank_driver jetank_teleop.launch

# Launch autonomous navigation (when implemented)
roslaunch jetank_navigation jetank_nav.launch

# Launch sock collection demo (when implemented)
roslaunch jetank_sock_collector sock_collector.launch
``` -->

## 🎯 Technical Challenges & Solutions

### Challenge 1: Hardware Abstraction Layer
**Solution**: Created modular drivers that abstract the JeTank hardware details, making them accessible through standard ROS interfaces.

### Challenge 2: Resource Constraints on Jetson Nano B01
**Solution**: Optimized code for the Jetson Nano's ARM architecture and 4GB memory, prioritizing efficient algorithms and parallel processing where possible.

### Challenge 3: ROS Melodic Compatibility
**Solution**: Ensured all packages and dependencies are compatible with ROS Melodic and the Ubuntu 18.04 environment on the Jetson Nano.

## 👨‍💻 About the Developer

This project aims to get more experience in robotics software integration, ROS architecture, Computer vision, and practical autonomous systems. By transforming a consumer robot kit into a fully ROS-compatible platform with advanced capabilities, I hope to increase my ability to bridge hardware and software in complex robotic systems, particularly on resource-constrained embedded platforms like the Jetson Nano.

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [Waveshare](https://github.com/waveshare/JETANK) for creating and open-sourcing the JeTank AI Kit
- The ROS community for their extensive documentation and support
- NVIDIA for the Jetson Nano platform
- The Construct for great ROS guides
