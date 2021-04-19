# Social Agents Simulation

## Packages setup

The packages needed in order to create a Social Agents Simulation including a robot are listed below:

- pedsim_ros (this same package)
- [ros_maps_to_pedsim](https://github.com/fverdoja/ros_maps_to_pedsim/tree/main)
- own robot description package (for Gazebo or RViz only)

## Scenarios

An scenario is an space where walls that enclose the agents are defined and there are different conigurations that can be defined. This is defined in an `.xml` file. Here is an example o one of them [ipa_apartment.xml](../pedsim_simulator/scenarios/ipa_apartment.xml).

### Obstacle

These are needed so that the agent knows where it can walk and where is it can't, this gives the repulsive forces and is mainly used to represent walls, obstacles are represented as prisms.

An example:

```xml
<obstacle x1="17.325" x2="17.275" y1="9.425" y2="9.475"/>
```

Definind the tags `x1`, `y1`, `x2` and `y2` corresponds to the dimension of the prism.

### Waypoints

These are the points in space where agents are declared to go. A series of waypoints are defined and then they are assigned in a certain order to the agents.

A waypoint is defined as:

```xml
<waypoint id="entrance_room" x="-2.2" y="-3.7" r="0.5"/>
```

The tags correspond to:

- `id`: name of the waypoint.
- `x`: it is the x coordinate where the waypoint is.
- `y`: is the y coordinate where the waypoint is.
- `r`: is the radius of the waypoint.

### Queue

A queue makes the agent make a line, one after another to pass through the space it is defined.

Example:

```xml
<queue id="klm" x="3.8" y="-5.2" direction="0"/>
```

`x` and `y` corresponds to the position and `direction` the angle in which the queue is made.

### Attraction

This one attracts the agents arbitrarily and makes them stare at it just like an attraction.

Example:

```xml
  <attraction id="robot_marketing" x="-12.2" y="-4.2" width="0.5" height="0.5" strength="2"/>
```

`width` and `height` are the dimension of the attraction, and `strength` is like how much attention demanding the attraction is.

### Agents

There are different types of agents, they can be defined as single agents or clustered/group agents. Here is an example about how they are defined:

```xml
<agent x="-10.2" y="-1.2" n="1" dx="2" dy="2" type="1">
    <addwaypoint id="ticket_dispensor"/>
    <addwaypoint id="middle_hall"/>
    <addwaypoint id="end_hall"/>
    <addwaypoint id="middle_hall"/>
</agent>
```

The different tags mean the following:

- `x`: it is the x coordinate in which the agent/s spawn.
- `y`: is the y coordinate in which the agent/s spawn.
- `n`: is the number of agents, if it is 1, then it is a single agents otherwise a group or cluster.
- `dx`: is the error there can be when spawning the agents in the x coordinate.
- `dy`:is the error there can be when spawning the agents in the y coordinate.
- `type`: there are the types:
  - ADULT/0 or CHILD/1: agent moves continuously.
  - ROBOT/2: agent is considered as a robot.
  - ELDER/3: agent stands still.

Inside the agents tag waypoints are defined with `addwaypoint` as seen in the example where `id` is the name of the waypoint.

In case a queue is wanted to be added, `addqueue` is used.

## Creating a scenario

As shown before, the scenario xml can be done by hand, however, the easiest way of creating a scenario depending on its complexity, is using the package ros_maps_to_pedsim. For that a `launch' file must be configured as in [ros_maps_to_pedsim.launch](../pedsim_simulator/launch/ros_maps_to_pedsim.launch).

There `map_path`, `map_name`, `scenario_path` and `scenario_name` must be configured so that the obstacles in the scenario are defined from a map and saved as an scenario in a defined directory.

Agents can also be added by configuring a `.yaml`.

## Launching scenario with robot (RViz only)

An example of how to do this is in [cob_pedsim_pedestrians_rviz_only.launch](../pedsim_simulator/launch/cob_pedsim_pedestrians_rviz_only.launch)

The more important things to configure are the following:

Whether a robot should be considered in simulation by the agents:

```xml
<arg name="with_robot" default="true"/>
```

The name of the scenario file in the package, should be located in a directory called `scenarios`, otherwise a path arg should be configured:

```xml
<arg name="scenario_file" default="ipa_apartment.xml"/>
```

If it is wanted to detect frozen agents:

```xml
<arg name="detect_frozen_agents" default="false"/>
```

Define the size of the walls which might not be visually correct:

```xml
<arg name="walls_scale" default="0.1"/>
```

If RViz is being used and a self publishing of the robot position will be done, it can be desired to publish or not the transform of the robots considered by `pedsim_simulator`:

```xml
<arg name="publish_tf" default="false"/>
```

The last thing to configure is the robot URDF description to be published as it is wanted, here the URDF of COB4 is published with the joints in an specific position. Also a TF publisher is launched to move the robot around the space.

```xml
<param name="robot_description" command="$(find xacro)/xacro --inorder '$(arg pkg_hardware_config)/robots/$(arg robot)/urdf/$(arg robot).urdf.xacro'" />
<param name="use_gui" value="$(arg gui)"/>
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
<rosparam file="$(find pedsim_simulator)/param/home_state_cob.yaml"/>
</node>
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

<node name="cob_tf_publisher" pkg="pedsim_simulator" type="cob_tf_publisher.py" output="screen" />
```
