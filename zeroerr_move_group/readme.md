# AroArm Move Group Interface
- [AroArm Move Group Interface](#aroarm-move-group-interface)
  - [Description](#description)
  - [Features](#features)
    - [Generate motion plan to join space goal `arm/JointSpaceGoal`](#generate-motion-plan-to-join-space-goal-armjointspacegoal)
    - [Generate motion plan to pose goal `arm/PointGoal`](#generate-motion-plan-to-pose-goal-armpointgoal)
    - [Execute motion plan `arm/Execute`](#execute-motion-plan-armexecute)
    - [Stop arm `arm/Stop`](#stop-arm-armstop)
    - [Clear current motion plan `arm/Clear`](#clear-current-motion-plan-armclear)


## Description

This package contains the nodes that utilize the Move Group Interface C++ API to generate pose targets from a custom topic interface.


## Features
### Generate motion plan to join space goal `arm/JointSpaceGoal`
The following command generates a motion plan at **10% of max speed** to joint angles:
- J1: $10\degree$
- J1: $45\degree$
- J1: $0\degree$
- J1: $0\degree$
- J1: $60\degree$
- J1: $90\degree$

```bash
ros2 topic pub --once /arm/jointspace zeroerr_msgs/msg/ArmJointSpace '{speed: 10, joint_pos_deg: [10, 45, 0, 0, 60, 90]}'
```


### Generate motion plan to pose goal `arm/PointGoal`

The following command generates a motion plan at **5% of max speed** to coordinates:
- x: $0.2$
- y: $-0.5$
- z: $0.8$

with orientation:
- x: $0.0$
- y: $0.0$
- z: $-\sqrt{2}/2$
- w: $\sqrt{2}/2$

```bash
ros2 topic pub --once /arm/point zeroerr_msgs/msg/ArmPoint '{speed: 5, pose: {position: {x: 0.2, y: -0.5, z: 0.8}, orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}}'
```


### Execute motion plan `arm/Execute`

To execute a generated motion plan:
```bash
ros2 topic pub --once /arm/Execute std_msgs/msg/Bool '{data: True}'
```


### Stop arm `arm/Stop`

To stop current execution of the arm:
```bash
ros2 topic pub --once /arm/stop std_msgs/msg/Bool '{data: True}'

# or
ros2 topic pub --once /arm/Execute std_msgs/msg/Bool '{data: False}'
```


### Clear current motion plan `arm/Clear`
To clear the current joint space or pose target:
```bash
ros2 topic pub --once /arm/Clear std_msgs/msg/Bool '{data: True}'
```
