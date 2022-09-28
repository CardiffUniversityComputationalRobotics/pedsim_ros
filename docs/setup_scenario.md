# Scenario Setup

A scenario file defines obstacles, waypoints and social agents that will be interacting in the space. Additionally, other interactions such as queues and interesting objects can be added. This is defined in a `.xml` file, example [ipa_apartment.xml](../pedsim_simulator/scenarios/ipa_apartment.xml).

Below all possible tags are explained:

## Obstacles

The social agents experience repulsive forces from defined obstacles, these are mainly used to represent walls with prism shapes.

An example:

```xml
<obstacle x1="17.325" x2="17.275" y1="9.425" y2="9.475"/>
```

The attributed `x1`, `y1`, `x2` and `y2` define where the obstacle starts and where it ends in the x-axis and y-axis.

### Waypoints

These are the points in space used as goals for the social agents.

A waypoint is defined as:

```xml
<waypoint id="entrance_room" x="-2.2" y="-3.7" r="0.5"/>
```

The attributes correspond to:

- `id`: name of the waypoint.
- `x`: position of the waypoint in the x-axis.
- `y`: position of the waypoint in the y-axis.
- `r`: radius tolerance of the waypoint.

### Queue

A queue acts as a waypoint, however, when several agents go to a queue, they make a line to pass one after another through the specified point.

Example:

```xml
<queue id="klm" x="3.8" y="-5.2" direction="0"/>
```

The attributes correspond to:

- `id`: name of the queue.
- `x`: position of the queue in the x-axis.
- `y`: position of the queue in the y-axis.
- `direction`: angle in which the queue is made (defined in degrees).

### Attraction

This element works as an attraction for the agents. Works in a random matter, when an agent passes near an attraction, there is a possibility of staying there staring at it for some time.

Example:

```xml
  <attraction id="robot_marketing" x="-12.2" y="-4.2" width="0.5" height="0.5" strength="2"/>
```

- `id`: name of the attraction.
- `x`: position of the attraction in the x-axis.
- `y`: position of the attraction in the y-axis.
- `width`: width of the attraction.
- `height`: height of the attraction.
- `strength`: how much the attraction influence the social agents.

### Agents

There are different types of agents, they can be defined as single agents or clustered/group agents. To specify which waypoints and queues the agent has to follow, these are specified inside the agent tag, normally these are followed in a loop and in an ordered manner,. Here is an example about how they are defined:

```xml
<agent x="-10.2" y="-1.2" n="1" dx="2" dy="2" type="1"  random="1" staticagent="1" orientation="30">
    <addwaypoint id="ticket_dispensor"/>
    <addwaypoint id="middle_hall"/>
    <addwaypoint id="end_hall"/>
    <addwaypoint id="middle_hall"/>
    <addqueue id="kq"/>
</agent>
```

The different attributes are defined below:

- `x`: x-axis coordinate in which the agent/s spawn.
- `y`: y-axis coordinate in which the agent/s spawn.
- `n`: number of spawned agents, if it is 1, then it is a single agent otherwise a group or cluster.
- `dx`: tolerance of spawn in the x-axis.
- `dy`: tolerance of spawn in the y-axis.
- `type`: there are different types:
  - ADULT/0 or CHILD/1: agent moves continuously.
  - ROBOT/2: agent is considered as a robot.
  - ELDER/3: agent may stand still sometimes.
- `random` (default: 0): if set to 1, the waypoints and queues are assigned randomly to the agent.
- `staticagent` (default: 0): if set to 1, the agent is static.
- `orientation` (default: 0): if the agent is static, the orientation can be defined.

**Note:** even if the agent is defined as static, define at least two dummy waypoints for the agent. Otherwise, currently the program crashes.

## Creating a scenario

As shown before, the scenario `.xml` can be done by hand, however, the easiest way of creating a scenario depending on its complexity, is using the package [ros_maps_to_pedsim](https://github.com/CardiffUniversityComputationalRobotics/ros_maps_to_pedsim). For that a `launch' file must be configured as in [ros_maps_to_pedsim.launch](<[../pedsim_simulator/launch/ros_maps_to_pedsim.launch](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/pedsim_simulator/launch/ros_maps_to_pedsim.launch)>).

There `map_path`, `map_name`, `scenario_path` and `scenario_name` must be configured so that the obstacles in the scenario are defined from a map and saved as an scenario in a defined directory. Agents can also be added by configuring a `.yaml`.
