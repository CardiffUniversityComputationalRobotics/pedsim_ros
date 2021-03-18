#!/usr/bin/env python3

import rospy
from pedsim_msgs.msg import AgentStates

agents_register_dict = {}


def is_frozen(id, actual_position):
    last_position = agents_register_dict[id][0]
    delta_position_x = abs(actual_position.x - last_position.x)
    delta_position_y = abs(actual_position.y - last_position.y)
    delta_position_z = abs(actual_position.z - last_position.z)

    if delta_position_x <= 1 and delta_position_y <= 1 and delta_position_z <= 1:
        return True
    else:
        return False


def delta_time(id):
    last_time = agents_register_dict[str(id)][1]
    if (rospy.get_rostime().secs - last_time) > 60:
        return True
    else:
        return False


def process_agents(agents_data):
    for agent in agents_data:
        if str(agent.id) in agents_register_dict:
            if is_frozen(agent.id, agent.position):
                if agents_register_dict[str(agent.id)][2] == "moving":
                    agent_last_info = agents_register_dict[str(agent.id)]
                    agent_last_info[1] = rospy.get_rostime().secs
                    agent_last_info[2] = "possibly_stuck"
                    agents_register_dict[str(agent.id)] = agent_last_info
                else:
                    if agents_register_dict[str(agent.id)][2] == "possibly_stuck":
                        if delta_time(agent.id):
                            agent_last_info = agents_register_dict[str(agent.id)]
                            agent_last_info[2] = "stuck"
                            agents_register_dict = agent_last_info

            else:
                agents_register_dict[str(agent.id)] = [
                    agent.position,
                    rospy.get_rostime().secs,
                    "moving",
                ]
        else:
            agents_register_dict[str(agent.id)] = [
                agent.pose.position,
                rospy.get_rostime().secs,
                "moving",
            ]


# class AgentsRegister(object):
#     def __init__(self):
#         print("hola")

#     def include_agent(self):
#         # TODO: agent adding should be included here.
#         pass


def agent_freezing_callback(data):
    input_msg = data.agent_states
    process_agents(input_msg)


def frozen_agent_detector():
    rospy.init_node("frozen_agent_detector_node", anonymous=True)
    rospy.Subscriber(
        "/pedsim_simulator/simulated_agents", AgentStates, agent_freezing_callback
    )
    rospy.spin()


if __name__ == "__main__":
    frozen_agent_detector()
