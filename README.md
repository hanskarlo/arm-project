# Arm Project MoveIt2 Repository

## About
This repository hosts the software for ROS2 integration with a 6 DoF 5kg ZeroErr arm for leveraging MoveIt2 features such as motion/trajectory planning, kinematic control, collision checking, and 3D perception.

The software was tested on an ASUS PN50 with
- Ryzen 5 4500U, 16GB Ram
- Realtek Semiconductor RTL8111/8168/8411 Gigabit Ethernet Controller
- Linux kernel 6.6.18 with PREEMPT_RT patch (6.6.18-rt23)

<br>

## Built with
[<img src="https://automaticaddison.com/wp-content/uploads/2023/10/ros2-iron.png" alt="ROS2 Iron Logo" width="240"/>](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html)
[<img src="https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/1717136168404-CV7O6LD1M56PNET8G161/JazzyJalisco_Final.png" alt="ROS2 Jazzy Logo" width="264"/>](https://docs.ros.org/en/jazzy/index.html)


[<img src="https://moveit.ros.org/assets/logo/moveit_logo-black.png" alt="MoveIt Logo" width="200"/>](https://github.com/ros-planning/moveit2)

[![EtherCat](https://gitlab.com/uploads/-/system/project/avatar/24894054/master-128.png?width=200)](https://gitlab.com/etherlab.org/ethercat)

<br>

## Getting started

### Prerequisites
* Linux Kernel with Real-time patch (Optional, **Recommended**)
* Linux Kernel with Xenomai 3 (Dovetail, Cobalt Core) (Optional, **Highly Recommended**)
* PC with Ethernet hardware
* Ubuntu 22.04 (Jammy Jellyfish) and ROS2 Iron
  * Or Ubuntu 24 (Noble) and ROS2 Jazzy
* MoveIt2 Humble or MoveIt2 Iron
* IgH EtherCAT Master for Linux
  * Compilation with Xenomai 3 kernel highly recommended
* cereal - A C++11 library for serialization

<br>

### Installation

#### ros2_control

Install the **ros2_control** software necessary for the hardware interface and controllers:
```bash
sudo apt install ros-<your ros distro>-ros2-control
sudo apt install ros-<your ros distro>-ros2-controllers
```

<br>

#### cereal

[**cereal**](https://uscilab.github.io/cereal/quickstart.html) is used for storing poses and trajectories by way of serialization and is used by the **arm_move_group** package.

Download **cereal** and place `cereal-1.3.2/include/cereal` into a folder where the project can find it. You can place it in `/usr/include/` (Recommended).
> :warning: Make sure to get cereal version **1.3.2**.
 
```bash
sudo cp -r ~/Downloads/cereal-1.3.2/include/cereal /usr/include/
```

<br>

#### Extra MoveIt2 Packages

If MoveIt2 software was compiled and built using the source code, skip to the [Build order](#build-order) section.
Otherwise, if MoveIt2 was installed via binary installation, the following packages will need to be installed:
```bash
source /opt/ros/<your ros distro>/setup.bash
sudo apt install ros-$ROS_DISTRO-moveit-servo
sudo apt install ros-$ROS_DISTRO-moveit-planners-chomp
sudo apt install ros-$ROS_DISTRO-moveit-visual-tools
sudo apt install ros-$ROS_DISTRO-pick-ik
```

<br>

#### EtherCAT Kernel Module

The IgH EtherLab EtherCAT Master software installation is only required if interfacing with the ZeroErr arm hardware. Refer to the [documentation](https://docs.etherlab.org/ethercat/1.5/pdf/ethercat_doc.pdf#chapter.9) on how to install it.


<br>
<br>


### Build order

Clone the repository **into your MoveIt2 workspace /src folder** 
```bash
cd /your_moveit2_ws/src
git clone https:/github.com/hanskarlo/arm-project.git
```

Source your workspace and build the packages
```bash
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

If using **real hardware**, start the EtherCAT Master module:
```bash
sudo /etc/init.d/ethercat start
```

<br>

### Motion Planning

Launch the arm configuration: 
```bash
# Simulated hardware
ros2 launch arm_config sim.launch.py

# Real hardware
ros2 launch arm_config hardware.launch.py
```

<br>

Run the `arm_move_group` interface (see the [readme](../zeroerr_arm/arm_move_group/README.md)) with 
- `visualize_trajectories`: false, 
- `servoing`: false.

<br>

If using **real hardware**, run the EtherCAT interface:
```bash
ros2 run arm_ethercat_interface arm_ethercat_interface
```

Wait until all joints are in Operation enabled state before attempting any motion plans.

> :bulb: In another terminal, run `ethercat slaves` to query the slave states, or watch the terminal.

> :exclamation: The node may take some time to configure the actuators to the EtherCAT OP state (see [Known Issues](#known-issues)) 

<br>



### Servoing

Launch the servo configuration:
```bash
# Simulation
ros2 launch arm_config servo.launch.py

# Real hardware
ros2 launch arm_config servo.launch.py hardware_type:=real
```

<br>

If motion plans are also desired, run the `arm_move_group` interface with
- `visualize_trajectories` false, 
- `servoing` true (see the [readme](../arm_project/arm_move_group/README.md)).

<br>

If using real hardware, run the EtherCAT interface:
```bash
ros2 run arm_ethercat_interface arm_ethercat_interface
```

Wait until all joints are in OP state before attempting any motion plans.

> :bulb: In another terminal, run `ethercat slaves` to query the slave states, or watch the terminal.

> :exclamation: The node may take some time to configure the actuators to the EtherCAT OP state (see [Known Issues](#known-issues)) 

<br>

#### Keyboard control servoing
For keyboard servoing, run the `servo_keyboard_control` node from the `arm_servo` package:
```bash
ros2 run arm_servo servo_keyboard_control
```

<br>

#### Game controller servoing
The ROS2 **Joy** package is used for servoing the arm via game controllers. 

Run the `Joy` package `game_controller_node`:
```bash
ros2 run joy game_controller_node
```

Connect a controller to the machine via Bluetooth, or USB. The connected controller should appear in the terminal output.

<br>

Run the 'arm_servo' package `game_controller` node:
```bash
ros2 run arm_servo game_controller
```

See the game controller servo control layout [here](../arm-project/arm_servo/Control_Layout.md).



<br>

## Known issues
* The arm_ethercat_interface node experiences turbulance when initially configuring the EtherCAT slaves (eRob actuators). This stems from a myriad of possible issues (hardware, communication frequency, control frequency).