#!/usr/bin/env python3

import math
from time import sleep
import threading
import rospy
from pedsim_msgs.msg import AgentStates
from dynamic_reconfigure.msg import Config
from std_msgs.msg import Float64


class FrozenAgentTime:
    """This class checks and publishes wether an agent is stuck or not"""

    def __init__(self):

        rospy.init_node("frozen_agent_time", anonymous=True)

        # Specific parameters that could be tuned
        self.time_callback_delay = rospy.get_param(
            "/frozen_agent_time/time_callback_delay", 0.1
        )
        self.time_radius_threshold = rospy.get_param(
            "/frozen_agent_time/time_radius_threshold", 0.1
        )
        self.time_publish_frequency = rospy.get_param(
            "/frozen_agent_time/time_publish_frequency", 1.0
        )

        # subcribers
        self._agents_status_sub = rospy.Subscriber(
            "/pedsim_simulator/simulated_agents",
            AgentStates,
            self.agent_freezing_callback,
            queue_size=1,
        )

        self._sim_parameters_sub = rospy.Subscriber(
            "pedsim_simulator/parameter_updates",
            Config,
            self.sim_parameters_callback,
            queue_size=1,
        )

        # publishers
        self._frozen_agents_time_pub = rospy.Publisher(
            "/frozen_agents_total_time", Float64, queue_size=10
        )

        # variables
        self.agents_register_dict = {}

        self.rate = rospy.Rate(self.time_publish_frequency)

        self.agent_callback_inc_msg = None
        self.sim_time = 0
        self.last_callback_time = self.sim_time

        self.update_rate = 25
        self.simulation_factor = 1

        self.frozen_agent_total_time_msg = Float64()
        self.frozen_agent_total_time_msg.data = 0
        self.frozen_agent_total_time = 0

    def sim_parameters_callback(self, data):
        """captures dynamic reconfigure parameters and changes them locally"""
        data = data.doubles
        self.update_rate = data[0].value
        self.simulation_factor = data[1].value

    def agent_freezing_callback(self, data):
        """AgentStates subscribe to get agents position"""
        if self.sim_time - self.last_callback_time > self.time_callback_delay:
            self.last_callback_time = self.sim_time
            input_msg = data.agent_states
            self.agent_callback_inc_msg = input_msg
            self.process_agents(input_msg)

    def sim_time_process(self):
        """keeps track of the time as the simulation does"""
        while True:
            self.sim_time += self.simulation_factor / self.update_rate
            sleep(1 / self.update_rate)

    def process_agents(self, agents_data):
        """Processes the agents data and carries frozen detection procedure."""
        for agent in agents_data:
            if str(agent.id) in self.agents_register_dict:

                if (
                    self.is_frozen(agent.id, agent.pose.position)
                    and agent.social_state != "Unknown"
                ):
                    if self.agents_register_dict[str(agent.id)][2] == "moving":
                        self.agents_register_dict[str(agent.id)] = [
                            agent.pose.position,
                            self.sim_time,
                            "frozen",
                        ]
                    else:
                        self.frozen_agent_total_time += (
                            self.sim_time - self.agents_register_dict[str(agent.id)][1]
                        )
                        self.agents_register_dict[str(agent.id)] = [
                            agent.pose.position,
                            self.sim_time,
                            "frozen",
                        ]
                else:
                    self.agents_register_dict[str(agent.id)] = [
                        agent.pose.position,
                        self.sim_time,
                        "moving",
                    ]
            else:
                self.agents_register_dict[str(agent.id)] = [
                    agent.pose.position,
                    self.sim_time,
                    "moving",
                ]

    def is_frozen(self, agent_id, actual_position):
        """return if agent is frozen based on its change of position,
        only X and Y position is considered"""
        last_position = self.agents_register_dict[str(agent_id)][0]
        delta_position_x = abs(actual_position.x - last_position.x)
        delta_position_y = abs(actual_position.y - last_position.y)

        rad_mov_value = math.sqrt(
            math.pow(delta_position_x, 2) + math.pow(delta_position_y, 2)
        )

        if rad_mov_value < self.time_radius_threshold:
            return True
        return False

    def start_time_publish(self):
        """Publishes the frozen status of the agents continously"""
        while not rospy.is_shutdown():
            self.frozen_agent_total_time_msg.data = self.frozen_agent_total_time
            self._frozen_agents_time_pub.publish(self.frozen_agent_total_time)
            self.rate.sleep()


if __name__ == "__main__":
    frozen_agent_time = FrozenAgentTime()
    sim_time_processor = threading.Thread(target=frozen_agent_time.sim_time_process)
    agents_detection_process = threading.Thread(
        target=frozen_agent_time.start_time_publish
    )
    sim_time_processor.setDaemon(False)
    agents_detection_process.setDaemon(False)
    sim_time_processor.start()
    agents_detection_process.start()
