#!/usr/bin/env python3

import math
from time import sleep
import threading
import rospy
from pedsim_msgs.msg import AgentStates, FrozenAgent, FrozenAgents
from dynamic_reconfigure.msg import Config


class FrozenAgentDetector:
    """This class checks and publishes wether an agent is stuck or not"""

    def __init__(self):

        rospy.init_node("frozen_agent_detector_node", anonymous=True)

        # Specific parameters that could be tuned
        self.callback_delay = rospy.get_param(
            "/frozen_agent_counter/callback_delay", 2.0
        )
        self.radius_threshold = rospy.get_param(
            "/frozen_agent_counter/radius_threshold", 1.0
        )
        self.time_threshold = rospy.get_param(
            "/frozen_agent_counter/time_threshold", 10.0
        )
        self.publish_frequency = rospy.get_param(
            "/frozen_agent_counter/publish_frequency", 1.0
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
        self._frozen_agents_pub = rospy.Publisher(
            "/frozen_agents", FrozenAgents, queue_size=10
        )

        # variables
        self.agents_register_dict = {}
        self.frozen_agents_list = []
        self.frozen_agents_msg = FrozenAgents()

        self.rate = rospy.Rate(self.publish_frequency)

        self.agent_callback_inc_msg = None
        self.sim_time = 0
        self.last_callback_time = self.sim_time

        self.update_rate = 25
        self.simulation_factor = 1

    def sim_parameters_callback(self, data):
        """captures dynamic reconfigura parameters and changes them locally"""
        data = data.doubles
        self.update_rate = data[0].value
        self.simulation_factor = data[1].value

    def agent_freezing_callback(self, data):
        """AgentStates subscribe to get agents position"""
        if self.sim_time - self.last_callback_time > self.callback_delay:
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
                        agent_last_info = self.agents_register_dict[str(agent.id)]
                        agent_last_info[1] = self.sim_time
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

        if rad_mov_value < self.radius_threshold:
            return True
        return False

    def delta_time(self, agent_id):
        """Outputs whether 60secs have passed considering the agent possibly stuck"""
        last_time = self.agents_register_dict[str(agent_id)][1]

        if (self.sim_time - last_time) > self.time_threshold:
            return True
        return False

    def start_detector(self):
        """Publishes the frozen status of the agents continously"""
        while not rospy.is_shutdown():
            self.frozen_agents_list = []
            for key, value in self.agents_register_dict.items():
                self.frozen_agents_list.append(FrozenAgent(int(key), value[2]))
            self.frozen_agents_msg.frozen_agents = self.frozen_agents_list

            # bug fix
            self._frozen_agents_pub.publish(self.frozen_agents_msg)

            try:
                for agent in self.agent_callback_inc_msg:
                    for key, value in self.agents_register_dict.items():
                        if value[2] == "stuck":
                            self.agents_register_dict[str(key)] = [
                                agent.pose.position,
                                self.sim_time,
                                "moving",
                            ]
            except Exception as _e:
                pass
                # print(_e)
            self.rate.sleep()


if __name__ == "__main__":
    frozen_agent_detector = FrozenAgentDetector()
    sim_time_processor = threading.Thread(target=frozen_agent_detector.sim_time_process)
    agents_detection_process = threading.Thread(
        target=frozen_agent_detector.start_detector
    )
    sim_time_processor.setDaemon(False)
    agents_detection_process.setDaemon(False)
    sim_time_processor.start()
    agents_detection_process.start()
