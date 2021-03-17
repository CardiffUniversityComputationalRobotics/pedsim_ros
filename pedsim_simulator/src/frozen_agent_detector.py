#!/usr/bin/env python3

import rospy
from pedsim_msgs.msg import AgentStates


# def resultant_force(forces):
# print("Forces are: ", forces)
# print(type(forces))
# print()
# resultant_force_x = (
#     forces.desired_force.x
#     + forces.obstacle_force.x
#     + forces.social_force.x
#     + forces.group_coherence_force.x
#     + forces.group_gaze_force.x
#     + forces.group_repulsion_force.x
#     + forces.random_force.x
# )
# resultant_force_y = (
#     forces.desired_force.y
#     + forces.obstacle_force.y
#     + forces.social_force.y
#     + forces.group_coherence_force.y
#     + forces.group_gaze_force.y
#     + forces.group_repulsion_force.y
#     + forces.random_force.y
# )
# resultant_force_z = (
#     forces.desired_force.z
#     + forces.obstacle_force.z
#     + forces.social_force.z
#     + forces.group_coherence_force.z
#     + forces.group_gaze_force.z
#     + forces.group_repulsion_force.z
#     + forces.random_force.z
# )
# print("Resultant X force:", resultant_force_x)
# print("Resultant Y force:", resultant_force_y)
# print("Resultant Z force:", resultant_force_z)
# print("Time of agent:", rospy.get_rostime().secs)


class AgentsRegister(object):
    def __init__(self):
        print("hola")

    def include_agent(self):
        # TODO: agent adding should be included here.
        pass


def agent_freezing_callback(data):
    input_msg = data.agent_states

    for i in input_msg:
        print("##############################")
        # print("Agent ID:", i.id)
        # print("Agent pose:", i.pose)
        # resultant_force(i.id, i.pose)


def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber(
        "/pedsim_simulator/simulated_agents", AgentStates, agent_freezing_callback
    )
    rospy.spin()


if __name__ == "__main__":
    listener()
