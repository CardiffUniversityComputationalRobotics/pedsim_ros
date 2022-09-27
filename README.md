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
git submodule update --init --recursive
cd ../..
catkin build -c  # or catkin_make
```

## Sample test

```
roslaunch pedsim_simulator simple_pedestrians.launch
```

## Usage instructions

![rosgraph](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/tree/noetic-devel/docs/rosgraph.png)

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
