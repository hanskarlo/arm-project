# Arm Project MoveIt2 Repository

## About
This repository hosts the software for ROS2 integration with a 6 DoF 5kg ZeroErr arm for leveraging MoveIt2 features such as motion/trajectory planning, kinematic control, collision checking, and 3D perception.

The software was tested on an ASUS PN50 with
- Ryzen 5 4500U, 16GB Ram
- Realtek Semiconductor RTL8111/8168/8411 Gigabit Ethernet Controller
- Linux kernel 6.6.18 with PREEMPT_RT patch (6.6.18-rt23)
<br>

## Built with

[<img src="https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/1653397531343-6M4IQ4JWDQV1SQ8W17UN/HumbleHawksbill_TransparentBG-NoROS.png" alt="MoveIt Logo" width="200"/>](https://docs.ros.org/en/humble/index.html)
[<img src="https://automaticaddison.com/wp-content/uploads/2023/10/ros2-iron.png" alt="MoveIt Logo" width="240"/>](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html)

[<img src="https://moveit.ros.org/assets/logo/moveit_logo-black.png" alt="MoveIt Logo" width="200"/>](https://github.com/ros-planning/moveit2)

[![EtherCat](https://gitlab.com/uploads/-/system/project/avatar/24894054/master-128.png?width=200)](https://gitlab.com/etherlab.org/ethercat)


## Getting started

### Prerequisites
* Linux Kernel with Real-time patch (**Optional**)
* Ethernet hardware
* Ubuntu 22.04 (Jammy Jellyfish)
* ROS2 Humble Hawksbill or ROS2 Iron Irwini
* MoveIt2 Humble or MoveIt2 Iron
* IgH EtherCAT Master for Linux

### Installation
Clone the repository **into your MoveIt2 workspace /src folder** 
```bash
cd /your_moveit2_ws/src
git clone https:/github.com/hanskarlo/arm-project.git
```

<br>

#### Build order

Source your workspace and build the packages
```bash
cd /your_moveit2_ws
source install/setup.bash
```

<br>

Build the robot description package containing the URDF file and associated meshes

```bash
colcon build --packages-select arm_description
```


<br>

Build the arm config package produced by the MoveIt2 setup assistant

```bash
colcon build --packages-select arm_config
```


<br>

Build the EtherCAT interface package and custom ros2_control hardware package
```bash
colcon build --packages-select arm_ethercat_interface arm_hardware 
```

<br>

Build the arm_msgs package containing the custom service interfaces
```bash
colcon build --packages-select arm_msgs
```

<br>

Build the rest of the packages:
- arm_move_group: Contains node that answers service calls to interface with the arm using the Move Group C++ API
- arm_servo: Utilizes keyboard input to servo arm utilizing MoveIt Servo
- arm_tests: Contains tests and example service calls in python

```bash
colcon build --packages-select arm_move_group arm_servo arm_tests
```

<br>

## Usage
Start the EtherCAT Master module:
```bash
sudo /etc/init.d/ethercat start
```

<br>

Run the arm_ethercat_interface node:
```bash
ros2 run arm_ethercat_interface arm_ethercat_interface
```

> :exclamation: The node may take some time to configure the actuators to the EtherCAT OP state (see [Known Issues](#known-issues)) 

<br>

Once all actuators have reached the **Operation enabled** state (CiA402 FSA), launch the MoveIt2 config: 
```bash
ros2 launch arm_config hardware.launch.py
```

<br>

## ToDo
- Explore different EtherCAT syncing techniques with the eRob actuators (i.e. DC Synchronization).
  - Consider placing EtherCAT interface software directly in ros2_control hardware interface


## Known issues
* The arm_ethercat_interface node experiences turbulance when initially configuring the EtherCAT slaves (eRob actuators). This stems from a myriad of possible issues (hardware, communication frequency, control frequency).
