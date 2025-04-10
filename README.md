Go2 Odometry
===

## Summary
Provide a simple state estimation for the unitree Go2 robot, via the [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package.
It also provides a simple node to convert unitree custom messages into "standard" ros messages and re-publish them on separate topics.


>[!IMPORTANT]
>Ressources concerning motion capture setup and building instructions for mocap attachements to the Go2 can be found [here](ressources/README.md)

## Launchfiles

### go2_odometry_switch.launch.py
"Main" launch file that takes a `mocap_type` argument to select between odometry sources.
The choices are : `use_full_odom` (default), `fake`, `mocap`

Command: 
```bash
ros2 launch go2_odometry go2_odometry_switch.launch.py odom_type:=<your choice>
```

- `mocap_type:=use_full_odom`
Calls the **go2_state_publisher.launch.py** launch file.

- `mocap_type:=fake`
Calls the **go2_fake_odom.launch.py** file.

- `mocap_type:=mocap`
Calls the **go2_mocap.launch.py** file.

Parameters available depending on the odometry used :
| Type of odometry choosen | | | | | |
|-- |-- | --| --|--|--|
|`use_full_odom`| No parameters |
|`fake` | base_height|
| `mocap`|base_frame|odom_frame|wanted_body|qualisys_ip|publishing_freq|

Details on each parameter are given in the launchfile description below.

---

### go2_full_odometry.launch.py
This file launches **go2_state_publisher.launch.py** detailled further down.
The other nodes launched are:

##### robot_localization/ekf_node
A standard ros2 node for running basic Extended Kalman Filters.

This Kalman is tuned to listen to:
* `/imu`: published in this case by the **go2_state_converter** node.
* `/tf`: populated in this case by the **robot_state_publisher** node.
* `/odometry/feet_pos` and `/odometry/feet_vel`: for XY speed and Z correction of the estimate. (See next node)

It then publishes on:
* `/tf`: The floating base pose estimation
* `/odometry/filtered`: The same pose estimate with covariances.

##### go2_odometry/feet_to_odom.py
Runs an inverse kinematics calculation on the robot joints to get a height estimate and a XY velocity estimate (assuming the feet are sticking to the ground).

This node subscribes to:
* `unitree_ros2/LowState`: For joint configuration and velocity, as-well as for **foot_force** to determine if the foot is in contact with the ground.

It the publishes on:
*`/odometry/feet_pos` and `/odometry/feet_vel`: For the ekf node to consume.

---
### go2_mocap.launch.py
Connects to a Qualisys Motion Capture and converts the data recevied in the expected output format of an odometry node of our Go2 stack. This allows to have a "perfect" odometry node that contains the ground truth data.

By default the node launches:
**go2_state_publisher.launch.py** (detailled further down).

##### go2_odometry/mocap_base_pose.py 
Node charged of the communication with the Qualisys Motion Capture system.

Published topics:
* `/odometry/filtered`: position of the robot base
* `/tf` : Transform between *odom_frame* (fixed) and *base_frame* (tied to the robot)

Ros parameters :
- `base_frame` (default: 'base') : name of the robot base frame
- `odom_frame` (default: 'odom') : name of the fixed frame 
- `wanted_body` (default: 'Go2') : name of the object to be tracked in the motion capture software
- `qualisys_ip` (default: 192.168.75.2) : IP used to communicate with the motion capture software
- `publishing_freq` (default: 110) : publishing frequency of the transform & odometry topics
- `mimic_go2_odometry` (default: 1) : defines the dehavior of the mocap node

>[!NOTE]
>If you'd like to use the motion capture as a ground truth run:
>```bash
>ros2 launch go2_odometry go2_mocap.launch.py mimic_go2_odometry:=0
>```
>The parameter `mimic_go2_odometry` changes the behavior of the node so that it can be used as a ground truth publisher.  
>What changes is:
>- **go2_state_publisher.launch.py** is not launched
>- **go2_odometry/mocap_base_pose.py** is launched but :
>    1. The topic `/odometry/filtered` is not published anymore
>    2. The tf now publishes a transform between `odom` and **`base_mocap`** (originally between `odom` and `base`)

---
### go2_fake_odometry.launch.py
Sets the robot to a fixed position (0,0,base_height) (base_height being a parameter) and fixed orientation (quaternion of 0,0,0,1). Used for debugging purposes.

Starts the following:
 **go2_state_publisher.launch.py** detailled further down.

##### go2_odometry/fake_odom.py
Fake odometry node.

Published topics:
* `/odometry/filtered`: position of the robot base
* `/tf` : Transform between *odom_frame* (fixed) and *base_frame* (tied to the robot)

Takes several ros parameters :
- base_frame (default: 'base') : name of the robot base frame
- odom_frame (default: 'odom') : name of the fixed frame 
- base_height (default: 0.30) : height of the robot base frame



 ---
### go2_state_publisher.launch.py
Starts the following nodes:
This file launches **go2_state_publisher.launch.py** detailled further down.

##### go2_odometry/state_converter_node.cpp
Listens to `unitree_ros2/LowState` messages and splits them into "standard" ros messages. It also re-arranges the joint order to match urdf order (which is not the order sent by the unitree robot).

The messages are timestamped using the host clock upon `/lowstate` msg reception (because the `unitree_ros2/LowState` msg is not timestamped)

Finally the published topics are the following:
* `/imu`: populated by `unitree_ros2/LowState`.`imu`
* `/joint_states`: populated by `unitree_ros2/LowState`.`motor_state`.<`d`/`dq`/...>
* `/clock`: populated by `utlidar/imu`

##### robot_state_publisher
Standard ros node that listens to `/joint_states` topic and publishes TF transform for all links of a robot.

Published topics:
* `/tf` and `/tf_static`: Link relative positions, populated from `/joint_states`.
* `/robot_description`:  topic for other nodes to get the urdf of the robot.

