#!/usr/bin/env python3
"""Code to compute and show forces gradient"""

import math
import rospy
from pedsim_msgs.msg import AgentStates


class GradientPublisher:
    """Class to compute and show force gradient."""

    rospy.init_node("force_gradient_node", anonymous=True)

    def __init__(self):

        self.agent_number = 0
        self.actual_total_sum_force = 0
        self.last_total_sum_forces = 0
        self.last_time_forces = 0
        self.actual_time_forces = 0
        self.gradient_forces = 0
        self.last_time_iteration = 0

        self._agents_status_sub = rospy.Subscriber(
            "/pedsim_simulator/simulated_agents",
            AgentStates,
            self.force_gradient_callback,
            queue_size=1,
        )

    def force_gradient_callback(self, msg):
        """Subscribe callback to compute gradient."""
        if rospy.get_rostime().secs - self.last_time_iteration >= 2:
            data = msg.agent_states
            for agent in data:
                if int(agent.id) == self.agent_number:

                    self.actual_time_forces = agent.header.stamp.secs

                    total_forces_x = (
                        agent.forces.desired_force.x
                        + agent.forces.obstacle_force.x
                        + agent.forces.social_force.x
                        + agent.forces.group_coherence_force.x
                        + agent.forces.group_gaze_force.x
                        + agent.forces.group_repulsion_force.x
                        + agent.forces.random_force.x
                    )

                    total_forces_y = (
                        agent.forces.desired_force.y
                        + agent.forces.obstacle_force.y
                        + agent.forces.social_force.y
                        + agent.forces.group_coherence_force.y
                        + agent.forces.group_gaze_force.y
                        + agent.forces.group_repulsion_force.y
                        + agent.forces.random_force.y
                    )

                    self.actual_total_sum_force = math.sqrt(
                        math.pow(total_forces_x, 2) + math.pow(total_forces_y, 2)
                    )

                    print("############")
                    print(rospy.get_rostime().nsecs)
                    self.gradient_forces = (
                        self.actual_total_sum_force - self.last_total_sum_forces
                    ) / (self.actual_time_forces - self.last_time_forces)

                    print("Total sum forces:", self.actual_total_sum_force)
                    print("Gradient of force:", self.gradient_forces)
                    self.last_total_sum_forces = self.actual_total_sum_force
                    self.last_time_forces = self.actual_time_forces
                    self.last_time_iteration = rospy.get_rostime().secs
                    if abs(self.gradient_forces) <= 0.06:
                        print("Frozen agent")


def start_node():
    """Method to start listening."""
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    gradient_node = GradientPublisher()
    start_node()
