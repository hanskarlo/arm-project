# Arm Move Group Interface
- [Arm Move Group Interface](#arm-move-group-interface)
  - [Description](#description)
  - [Launch](#launch)
    - [Parameters](#parameters)
    - [Example](#example)
  - [Features](#features)
    - [Generate motion plan to joint space goal `arm/JointSpaceGoal`](#generate-motion-plan-to-joint-space-goal-armjointspacegoal)
    - [Generate motion plan to pose goal `arm/PoseGoal`](#generate-motion-plan-to-pose-goal-armposegoal)
    - [Generate motion plan via array of end-effector pose waypoints (i.e. trajectory) `arm/PoseGoalArray`](#generate-motion-plan-via-array-of-end-effector-pose-waypoints-ie-trajectory-armposegoalarray)
    - [Execute motion plan `arm/Execute`](#execute-motion-plan-armexecute)
    - [Stop arm `arm/Stop`](#stop-arm-armstop)
    - [Clear current motion plan `arm/Clear`](#clear-current-motion-plan-armclear)
  - [Notes](#notes)
    - [Motion Planners](#motion-planners)


## Description

This package hosts the node that utilizes the Move Group C++ Interface to generate motion plans from a custom service interface.

The custom service interface is outlined in the **arm_msgs** package. Example service calls can be found in the **arm_tests** package.

## Launch

### Parameters
`visualize_trajectories` Boolean parameter to trace end effector trajectory path of motion plan. 

> :exclamation: Will need a Marker Array module in RVIZ2 subscribed to `/arm_marker_array`, otherwise code will be blocked.

<br>

`servoing` Boolean parameter denoting if arm in servo mode.


### Example
To launch the arm move group interface:
```bash
ros2 launch arm_move_group move.group.launch.py visualize_trajectories:=false servoing:=false
```

<br>


## Features
### Generate motion plan to joint space goal `arm/JointSpaceGoal`
The following command generates a motion plan at **10% of max speed** to joint angles:
- J1: $10\degree$
- J2: $45\degree$
- J3: $0\degree$
- J4: $0\degree$
- J5: $60\degree$
- J6: $90\degree$

```bash
ros2 service call /arm/JointSpaceGoal arm_msgs/srv/JointSpaceGoal '{speed: 10, joint_pos_deg: [10, 45, 0, 0, 60, 90]}'
```
<br>
<br>

### Generate motion plan to pose goal `arm/PoseGoal`

The following command generates a motion plan at **30% of max speed** to coordinates:
- x: $0.2$m
- y: $-0.5$m
- z: $0.8$m

with quaternion orientation:
- x: $0.0$
- y: $0.0$
- z: $-\sqrt{2}/2$
- w: $\sqrt{2}/2$

```bash
ros2 service call /arm/PoseGoal arm_msgs/srv/PoseGoal '{speed: 30, pose: {position: {x: 0.2, y: -0.5, z: 0.8}, orientation: {x: 0.0, y: 0.0, 
z: -0.7071, w: 0.7071}}}'
```

<br>
<br>


### Generate motion plan via array of end-effector pose waypoints (i.e. trajectory) `arm/PoseGoalArray`

<br>

The ros2 service is of type `PoseTargetArray` which has parameters:
- `type`:
  - `'linear'` for linear movements
  - `'arc'` for curved/circular movements
- `step_size`: Interpolation between points (default $0.01$m)
- `jump_threshold`: Limits jump between generated points (default $0.0$m (ignored))
- `waypoints`: Array of type `geometry_msgs/Pose` messages -- desired waypoints the end effector to reach sequentially

<br>
<br>

The following command generates a linear motion plan with the following waypoints assuming **orientation fixed at identity**, 1cm max step size, and jump threshold ignored:

- Waypoint 1:
  - x: $0.2$m
  - y: $0.0$m
  - z: $0.0$m
- Waypoint 2:
  - x: $0.2$m
  - y: $0.4$m
  - z: $0.3$m

```bash
ros2 service call /arm/PoseGoalArray arm_msgs/srv/PoseGoalArray '{type: 'linear', step_size: 0.01, jump_threshold: 0.0, waypoints: [{position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, {position: {x: 0.2, y: 0.4, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}'
```

<br>
<br>

The following command generates a curved motion plan creating a quarter-circle arc in the $xz$ plane with radius $0.1$m assuming **start position at origin**, **orientation fixed at identity**, 1cm max step size, and jump threshold ignored:


- Center of motion
  - x: $0.01$m
  - y: $0.0$
  - z: $0.0$

- Endpoint
  - x: $0.01$m
  - y: $0.0$
  - z: $0.01$m

<br>

```bash
ros2 service call /arm/PoseGoalArray arm_msgs/srv/PoseGoalArray '{type: 'arc', step_size: 0.01, jump_threshold: 0.0, waypoints: [{position: {x: 0.01, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},{position: {x: 0.01, y: 0.0, z: 0.01}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}'
```

<br>

### Execute motion plan `arm/Execute`

To execute a generated motion plan:
```bash
ros2 service call /arm/Execute std_srvs/srv/Trigger
```

<br>

### Stop arm `arm/Stop`

To stop current execution of the arm:
```bash
ros2 service call /arm/Stop std_srvs/srv/Trigger
```

<br>

### Clear current motion plan `arm/Clear`
To clear the current motion plan:
```bash
ros2 topic pub --once /arm/Clear std_msgs/msg/Bool '{data: True}'
```

<br>

## Notes
### Motion Planners
The arm move group interface node utilizes the STOMP planner for linear movements in space, and uses the PILZ ('CIRC') industrial motion planner for circular/arc movements.

> :warning: STOMP planner only accepts **joint space** goals.