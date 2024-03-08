# ZeroErr Arm MoveIt2 Repository

## About
This repository hosts the software for ROS2 integration with a 6 DoF 5kg ZeroErr arm for leveraging MoveIt2 features such as motion/trajectory planning, kinematic control, collistion checking, and 3D perception.

The software was tested on a ASUS PN50 with
- Ryzen 5 4500U, 16GB Ram
- Realtek Semiconductor RTL8111/8168/8411 Gigabit Ethernet Controller
- Linux kernel 6.6.18 with PREEMPT_RT patch (6.6.18-rt23)
<br>

## Built with

[<img src="https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/1653397531343-6M4IQ4JWDQV1SQ8W17UN/HumbleHawksbill_TransparentBG-NoROS.png" alt="MoveIt Logo" width="200"/>](https://docs.ros.org/en/humble/index.html)

[<img src="https://moveit.ros.org/assets/logo/moveit_logo-black.png" alt="MoveIt Logo" width="200"/>](https://github.com/ros-planning/moveit2)

[![EtherCat](https://gitlab.com/uploads/-/system/project/avatar/24894054/master-128.png?width=200)](https://gitlab.com/etherlab.org/ethercat)


## Getting started

### Prerequisites
* Linux Kernel with Real-time patch (**Optional**)
* Ethernet hardware
* Ubuntu 22.04 (Jammy Jellyfish)
* ROS2 Humble
* MoveIt2
* IgH EtherCAT Master for Linux

### Installation
Clone the repository **into your MoveIt2 workspace /src folder** 
```
cd /your_moveit2_ws/src
git clone https:/github.com/hanskarlo/zeroerr_arm.git
```

Source your workspace and build the packages
```
cd /your_moveit2_ws
source install/setup.bash
colcon build
```

## Usage
Run the zeroerr_interface node:
```bash
ros2 run zeroerr_interface zeroerr_interface
```

> :exclamation: The node may take some time to configure the actuators to the EtherCAT OP state (see [Known Issues](#known-issues)) 

<br>

Once all actuators have reached the **Operation enabled** state (CiA402 FSA), launch the MoveIt2 config: 
```bash
ros2 launch zeroerr_config hardware.launch.py
```

## ToDo
- Support for ROS2 Iron, and consequently, newer MoveIt2 versions/features.
- Explore different EtherCAT syncing techniques with the eRob actuators (i.e. DC Synchronization).


## Known issues
* The zeroerr_interface node experiences turbulance when initially configuring the EtherCAT slaves (eRob actuators). This stems from a myriad of possible issues (hardware, communication frequency, control frequency).
