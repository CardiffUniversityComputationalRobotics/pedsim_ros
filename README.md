# Pedestrian Simulator

<img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/crowd1.png width=400/> | <img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/costmap.png width=400/>

ROS packages for a 2D pedestrian simulator based on social force
model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf). The implementation is based on an extended version of Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library which has been extended to include additional behaviors and activities. This packages is useful for robot navigation experiments with crowded scenes which are hard to acquire in practice.

## Features

- Individual walking using social force model for very large crowds in real time
- Group walking using the extended social force model
- Individual static social agents
- Individual frame following social agent
- Social activities simulation
- Sensors simulation (point clouds in robot frame for people and walls)
- XML based scene design
- Extensive visualization using Rviz
- Option to connect with gazebo for physics reasoning

## Requirements

- ROS with the visualization stack (currently tested on `hydro`, `indigo`, `kinetic` ). For `melodic`, see the branch `melodic-devel`
- C++11 compiler and for `noetic`, see the branch `noetic-devel`.

## Installation

The default version is now `noetic-devel`.

```
cd [workspace]/src
git clone https://github.com/srl-freiburg/pedsim_ros.git
cd pedsim_ros
catkin build  # or catkin_make
```

## Sample test

```
roslaunch pedsim_simulator simple_pedestrians.launch
```

## Usage instructions

![rosgraph](https://raw.githubusercontent.com/CardiffUniversityComputationalRobotics/pedsim_ros/noetic-devel/docs/rosgraph.png)

**Note:** currently `/pedbot` is not working.

### Simulate pedestrians

To start simulating people, the launch file `simulator.launch` in the package `pedsim_simulator` is used. This launch file starts a `/pedsim_simulator` node. The published and subscribed topics are described below:

#### Published topics

- /pedsim_simulator/parameter_descriptions ([dynamic_reconfigure/ConfigDescription](http://docs.ros.org/en/noetic/api/dynamic_reconfigure/html/msg/ConfigDescription.html))
- /pedsim_simulator/parameter_updates ([dynamic_reconfigure/ConfigDescription](http://docs.ros.org/en/noetic/api/dynamic_reconfigure/html/msg/ConfigDescription.html))
- /pedsim_simulator/robot_position ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
- /pedsim_simulator/simulated_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/AgentStates.msg))
- /pedsim_simulator/simlated_groups ([pedsim_msgs/AgentGroups](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/AgentGroups.msg))
- /pedsim_simulator/simulated_walls ([pedsim_msgs/LineObstacles](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/LineObstacles.msg))
- /pedsim_simulator/simulated_waypoints ([pedsim_msgs/Waypoints](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/Waypoints.msg))

#### Subscribed topics

- /tf ([tf2_msgs/TFMessage](http://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html))

#### Services

- /pedsim_simulator/pause_simulation ([std_srvs/Empty](http://docs.ros.org/en/api/std_srvs/html/srv/Empty.html))
- /pedsim_simulator/unpause_simulation ([std_srvs/Empty](http://docs.ros.org/en/api/std_srvs/html/srv/Empty.html))

#### Parameters

- output (string, default: screen)

  &emsp;If output desired from the node, can either be `screen` or `log`.

- kbd_teleop (bool, default: false)

  &emsp;If set to True, a keyboard teleop is initialized in the terminal.

- rqt_teleop (bool, default: false)

  &emsp;If set to True, a new rqt teleop window is opened.

- scene_file (string, default: pedsim_simulator/scenarios/social_contexts.xml)

  &emsp;Scenario file in which social agents are defined along with their waypoints and obstacles as well as other configurations. To see how a scenario file is defined, check [scenario setup](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/docs/setup_scenario.md).

- with_robot (bool, default: true)

  &emsp;If robot should be spawned and considered by the social agents.

- simulation_factor (double, default: 1)

  &emsp;The time factor between real time and the simulation.

- update_rate (double, default: 25)

  &emsp;Frequency of the social agents state update.

- default_queue_size (int, default: 10)

  &emsp;Queue size of the publishers in the node.

- max_robot_speed (double, default: 1.5)

  &emsp;Maximum velocity of the robot if controlled by the social force model in pedsim. (Currently not working)

- robot_mode (int, default: 1)

  &emsp;This value can be: 0 - controlled | 1 - teleoperation | 3 - social drive.

- enable_groups (bool, default: true)

  &emsp;If set to true, group interactions are simulated and published.

- pose_initial_x (double, default: 5.0)

  &emsp;The initial position of the robot in the x-axis.

- pose_initial_y (double, default: 5.0)

  &emsp;The initial position of the robot in the y-axis.

- pose_initial_theta (double, default: 0.0)

  &emsp;The initial orientation of the robot.

- detect_frozen_agents (bool, default: true)

  &emsp;If set to true, the social agents are analyzed and identified when they have been stuck in a place and then assigned to a random waypoint.

- force_obstacle (double, default: 10.0)

  &emsp;Defines the force factor magnitude coming from obstacles.

- sigma_obstacle (double, default: 0.2)

  &emsp;Corresponds to the range of interaction with obstacles.

- force_social (double, default: 5.1)

  &emsp;Defines the force factor magnitude coming from social agents.

- force_group_gaze (double, default: 3.0)

  &emsp;Not used.

- force_group_coherence (double, default: 3.0)

  &emsp;Not used.

- force_group_repulsion (double, default: 1.0)

  &emsp;Not used.

- force_random (double, default: 0.1)

  &emsp;Not used.

- force_wall (double, default: 2.0)

  &emsp;Not used.

- frame_id (string, default: odom)

  &emsp;Main reference frame for the position of social agents.

- robot_base_frame_id (string, default: base_footprint)

  &emsp;Main robot frame to be considered as the robot's position by pedsim.

- obstacle_offset_x (double, default: 0.0)

  &emsp;Walls and other obstacles position offset in the x-axis.

- obstacle_offset_y (double, default: 0.0)

  &emsp;Walls and other obstacles position offset in the x-axis.

- world_frame (string, default: odom)

  &emsp;Main frame in the tree.

- robot_frame (string, default: base_footprint)

  &emsp;Robot frame transform published by pedsim.

- publish_tf (string, default: true)

  &emsp;Publishes transform of the robot_frame, if set to false, the transform has to be published manually.

#### Publish agents transforms

To have the transforms, these can be obtained running the `pedsim_frames_publisher.py` from package `pedsim_simulator` as follows:

```xml
<node pkg="pedsim_gazebo_plugin" type="spawn_pedsim_agents.py" name="spawn_pedsim_agents" output="screen" />
```

### Spawn agents in gazebo

To make social agents appear in gazebo and move around, some other steps are required. First, run the `spawn_pedsim_agents.py` node from package `pedsim_gazebo_plugin` as follows:

```xml
<node pkg="pedsim_gazebo_plugin" type="spawn_pedsim_agents.py" name="spawn_pedsim_agents" output="screen" />
```

Second, in the `*.world` file used, the following an Actor Poses plugin must be added as follows (check [empy_world.py](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/pedsim_simulator/worlds/empty_default.world)):

```xml
<plugin name="ActorPosesPlugin" filename="libActorPosesPlugin.so"></plugin>
```

### Visualize pedestrians

For visualization of the social agents in rviz, it is important to run the node `pedsim_visualizer_node` from the package `pedsim_visualizer`.

#### Published topics

- /pedsim_visualizer/forces ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
- /pedsim_visualizer/tracked_groups ([pedsim_msgs/TrackedGroups](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/TrackedGroups.msg))
- /pedsim_visualizer/tracked_persons ([pedsim_msgs/TrackedPersons](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/TrackedPersons.msg))
- /pedsim_visualizer/walls ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
- /pedsim_visualizer/waypoints ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))

#### Subscribed topics

- /pedsim_simulator/simulated_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/AgentStates.msg))
- /pedsim_simulator/simlated_groups ([pedsim_msgs/AgentGroups](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/AgentGroups.msg))
- /pedsim_simulator/simulated_walls ([pedsim_msgs/LineObstacles](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/LineObstacles.msg))
- /pedsim_simulator/simulated_waypoints ([pedsim_msgs/Waypoints](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/master/pedsim_msgs/msg/Waypoints.msg))

## Licence

The core `libpedsim` is licensed under LGPL. The ROS integration and extensions are licensed under BSD.

## Developers

- Billy Okal
- Timm Linder

## Contributors

- Dizan Vasquez
- Sven Wehner
- Omar Islas
- Luigi Palmieri

The package is a **work in progress** mainly used in research prototyping. Pull requests and/or issues are highly encouraged.

## Acknowledgements

These packages have been developed in part during the EU FP7 project [SPENCER](spencer.eu)
