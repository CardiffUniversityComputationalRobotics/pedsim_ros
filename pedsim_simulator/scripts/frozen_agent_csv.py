#!/usr/bin/env python3

import rospy
from pedsim_msgs.msg import AgentStates, FrozenAgent, FrozenAgents
import math


class FrozenAgentCounter:
    """This class manages the state of the agents based on it position and time"""

    def __init__(self):

        rospy.init_node("frozen_agent_csv_writer", anonymous=True)

        # dict to know if agent was frozen before
        self.agents_register_dict = {}

        # frozen agents counter
        self.frozen_agents_counter = 0

        # subcribers
        self._agents_status_sub = rospy.Subscriber(
            "/frozen_agents",
            FrozenAgents,
            self.agent_freezing_callback,
            queue_size=1,
        )

    def agent_freezing_callback(self, data):
        """AgentStates subscribe to get agents position"""
        agents_status = data.frozen_agents
        for agent in agents_status:
            if str(agent.id) in self.agents_register_dict:
                if (
                    agent.is_frozen == "stuck"
                    and self.agents_register_dict[str(agent.id)] == "moving"
                ):
                    self.frozen_agents_counter += 1
                    self.agents_register_dict[str(agent.id)] == "stuck"
                elif agent.is_frozen == "moving":
                    self.agents_register_dict[str(agent.id)] == "moving"
            else:
                self.agents_register_dict[str(agent.id)] = "moving"