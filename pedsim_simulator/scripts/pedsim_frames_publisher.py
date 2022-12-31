#!/usr/bin/env python3

import rospy
import tf
from pedsim_msgs.msg import AgentStates


class AgentStatesTfBroadcaster(object):
    """Publishes AgentStates transforms"""

    def __init__(self):

        self.agents_list = []

        # SUBSCRIBERS
        self.agents_sub = rospy.Subscriber(
            "pedsim_simulator/simulated_agents",
            AgentStates,
            self.agents_register_callback,
        )

        self.br = tf.TransformBroadcaster()

    def agents_register_callback(self, data):
        self.agents_list = data.agent_states

    def run(self):
        while not rospy.is_shutdown():

            agents_to_erase = []

            for i in range(0, len(self.agents_list)):

                if rospy.Time.now().secs - self.agents_list[i].header.stamp.secs > 1000:
                    agents_to_erase.append(i)

                self.br.sendTransform(
                    (
                        self.agents_list[i].pose.position.x,
                        self.agents_list[i].pose.position.y,
                        0,
                    ),
                    (
                        self.agents_list[i].pose.orientation.x,
                        self.agents_list[i].pose.orientation.y,
                        self.agents_list[i].pose.orientation.z,
                        self.agents_list[i].pose.orientation.w,
                    ),
                    rospy.Time.now(),
                    "agent_" + str(self.agents_list[i].id),
                    self.agents_list[i].header.frame_id,
                )

            for i in agents_to_erase:
                del self.agents_list[i]
            rospy.sleep(0.005)
            # rospy.spin()


if __name__ == "__main__":
    rospy.init_node("pedsim_tf_broadcaster")
    ped_tf_broadcaster = AgentStatesTfBroadcaster()
    ped_tf_broadcaster.run()
