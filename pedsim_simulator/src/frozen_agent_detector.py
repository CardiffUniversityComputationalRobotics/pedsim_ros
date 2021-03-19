#!/usr/bin/env python3

import rospy
from pedsim_msgs.msg import AgentStates, FrozenAgent, FrozenAgents


class FrozenAgentDetector:
    """This class manages the state of the agents based on it position and time"""

    def __init__(self):

        # Specific parameters that could be tuned
        self.callback_delay = rospy.get_param("callback_delay", 2)
        self.distance_threshold = rospy.get_param("distance_threshold", 0.5)
        self.time_threshold = rospy.get_param("time_threshold", 10)
        self.publish_frequency = rospy.get_param("publish_frequency", 1)

        rospy.init_node("frozen_agent_detector_node", anonymous=True)

        # subcribers
        self._agents_status_sub = rospy.Subscriber(
            "/pedsim_simulator/simulated_agents",
            AgentStates,
            self.agent_freezing_callback,
            queue_size=1,
        )

        # publishers
        self._frozen_agents_pub = rospy.Publisher(
            "/frozen_agents", FrozenAgents, queue_size=10
        )

        # variables
        self.agents_register_dict = {}
        self.frozen_agents_list = []
        self.frozen_agents_msg = FrozenAgents()

        self.last_callback_time = rospy.get_rostime().secs

        self.rate = rospy.Rate(self.publish_frequency)

    def agent_freezing_callback(self, data):
        """AgentStates subscribe to get agents position"""
        if rospy.get_rostime().secs - self.last_callback_time > self.callback_delay:
            self.last_callback_time = rospy.get_rostime().secs
            input_msg = data.agent_states
            self.process_agents(input_msg)

    def process_agents(self, agents_data):
        """Processes the agents data and carries frozen detection procedure."""
        for agent in agents_data:
            if str(agent.id) in self.agents_register_dict:
                if self.is_frozen(agent.id, agent.pose.position):
                    if self.agents_register_dict[str(agent.id)][2] == "moving":
                        agent_last_info = self.agents_register_dict[str(agent.id)]
                        agent_last_info[1] = rospy.get_rostime().secs
                        agent_last_info[2] = "possibly_stuck"
                        self.agents_register_dict[str(agent.id)] = agent_last_info
                    else:
                        if (
                            self.agents_register_dict[str(agent.id)][2]
                            == "possibly_stuck"
                        ):
                            if self.delta_time(agent.id):
                                agent_last_info = self.agents_register_dict[
                                    str(agent.id)
                                ]
                                agent_last_info[2] = "stuck"
                                self.agents_register_dict[
                                    str(agent.id)
                                ] = agent_last_info

                else:
                    self.agents_register_dict[str(agent.id)] = [
                        agent.pose.position,
                        rospy.get_rostime().secs,
                        "moving",
                    ]
            else:
                self.agents_register_dict[str(agent.id)] = [
                    agent.pose.position,
                    rospy.get_rostime().secs,
                    "moving",
                ]

    def is_frozen(self, agent_id, actual_position):
        """return if agent is frozen based on its change of position,
        only X and Y position is considered"""
        last_position = self.agents_register_dict[str(agent_id)][0]
        delta_position_x = abs(actual_position.x - last_position.x)
        delta_position_y = abs(actual_position.y - last_position.y)

        if (
            delta_position_x <= self.distance_threshold
            and delta_position_y <= self.distance_threshold
        ):
            return True
        return False

    def delta_time(self, agent_id):
        """Outputs whether 60secs have passed considering the agent possibly stuck"""
        last_time = self.agents_register_dict[str(agent_id)][1]

        if (rospy.get_rostime().secs - last_time) > self.distance_threshold:
            return True
        return False

    def start_detector(self):
        """Publishes the frozen status of the agents continously"""
        while not rospy.is_shutdown():
            self.frozen_agents_list = []
            for key, value in self.agents_register_dict.items():
                self.frozen_agents_list.append(FrozenAgent(int(key), value[2]))
            self.frozen_agents_msg.frozen_agents = self.frozen_agents_list

            self._frozen_agents_pub.publish(self.frozen_agents_msg)
            self.rate.sleep()


if __name__ == "__main__":
    frozen_agent_detector = FrozenAgentDetector()
    frozen_agent_detector.start_detector()
