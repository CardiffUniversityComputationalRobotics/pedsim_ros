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
        for i in self.agents_list:

            self.br.sendTransform(
                (i.pose.position.x, i.pose.position.y, 0),
                (
                    i.pose.orientation.x,
                    i.pose.orientation.y,
                    i.pose.orientation.z,
                    i.pose.orientation.w,
                ),
                rospy.Time.now(),
                "agent_" + str(i.id),
                i.header.frame_id,
            )

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node("pedsim_tf_broadcaster")
    ped_tf_broadcaster = AgentStatesTfBroadcaster()
    ped_tf_broadcaster.run()
