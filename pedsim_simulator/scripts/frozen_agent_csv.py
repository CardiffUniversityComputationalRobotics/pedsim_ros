#!/usr/bin/env python3

import rospy
from pedsim_msgs.msg import AgentStates, FrozenAgent, FrozenAgents
import math
import csv


class FrozenAgentCounter:
    """This class manages the state of the agents based on it position and time"""

    def save_value_csv(self):
        # TODO: create or modify csv adding value
        pass
        with open(
            self.csv_dir + self.solution_type + self.csv_name, "a", newline=""
        ) as csvfile:
            spamwriter = csv.writer(
                csvfile, delimiter=" ", quotechar="|", quoting=csv.QUOTE_MINIMAL
            )
            spamwriter.writerow(
                [
                    "first_test",
                    str(self.frozen_agents_counter),
                ]
            )

    def __init__(self):

        rospy.init_node("frozen_agent_csv_writer", anonymous=True)

        rospy.on_shutdown(self.save_value_csv)

        # dict to know if agent was frozen before
        self.agents_register_dict = {}

        # frozen agents counter
        self.frozen_agents_counter = 0

        # ! directory of csv files
        self.csv_dir = rospy.get_param(
            "/frozen_agent_csv/csv_dir",
            "/home/sasm/people_sim_ws/src/pedsim_ros/pedsim_simulator/metrics/",
        )

        self.solution_type = rospy.get_param(
            "/frozen_agent_csv/solution_type", "position/"
        )

        self.csv_name = rospy.get_param("/frozen_agent_csv/csv_name", "first_test.csv")

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


if __name__ == "__main__":
    csv_counter_saver = FrozenAgentCounter()
    while True:
        rospy.spin()
