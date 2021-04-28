#!/usr/bin/env python

import roslib
import rospy
import tf
from geometry_msgs.msg import Pose, TransformStamped
from dynamic_reconfigure.msg import Config
from time import sleep

roslib.load_manifest("pedsim_simulator")

# all poses
pose1 = Pose()
pose1.position.x = -11.7
pose1.position.y = 12.8
pose1.orientation.z = -1.57

pose2 = Pose()
pose2.position.x = -11.7
pose2.position.y = 0.0
pose2.orientation.z = -1.57

pose3 = Pose()
pose3.position.x = -11.7
pose3.position.y = 0.0
pose3.orientation.z = -1.57 / 2

pose4 = Pose()
pose4.position.x = -8
pose4.position.y = -3.75
pose4.orientation.z = -1.575 / 2

pose5 = Pose()
pose5.position.x = -8
pose5.position.y = -3.75

pose6 = Pose()
pose6.position.x = 2.5
pose6.position.y = -3.75

pose7 = Pose()
pose7.position.x = 2.5
pose7.position.y = -3.75
pose7.orientation.z = 3.1416

pose8 = Pose()
pose8.position.x = -8
pose8.position.y = -3.75
pose8.orientation.z = 3.1416

pose9 = Pose()
pose9.position.x = -8
pose9.position.y = -3.75
pose9.orientation.z = 3.1416 * 3 / 4

pose10 = Pose()
pose10.position.x = -11.7
pose10.position.y = 0.0
pose10.orientation.z = 3.1416 * 3 / 4

pose11 = Pose()
pose11.position.x = -11.7
pose11.position.y = 0.0
pose11.orientation.z = 1.57

pose12 = Pose()
pose12.position.x = -11.7
pose12.position.y = 12.8
pose12.orientation.z = 1.57

pose13 = Pose()
pose13.position.x = -11.7
pose13.position.y = 12.8
pose13.orientation.z = -1.57

poses_list = [
    [pose1, 20],  # recta
    [pose2, 4],  # giro
    [pose3, 8],  # recta
    [pose4, 5],  # giro
    [pose5, 15],  # recta
    [pose6, 8],  # giro regreso
    [pose7, 15],  # recta
    [pose8, 4],  # giro
    [pose9, 9],  # recto
    [pose10, 3],  # giro
    [pose11, 20],  # recta
    [pose12, 5],  # giro
    [pose13, 5],  # no se da tiempo
]


class COB_TF_Publisher:
    def __init__(self):
        rospy.init_node("simulator_tf_broadcaster")
        self.rqt_parameters_sub = rospy.Subscriber(
            "/pedsim_simulator/parameter_updates",
            Config,
            self.parameters_callback,
            queue_size=1,
        )
        self.update_rate = 25
        self.simualtion_factor = 1

        self.current_x_position = pose1.position.x
        self.current_y_position = pose1.position.y
        self.current_z_orientation = pose1.orientation.z

        self.t = TransformStamped()
        self.t.header.frame_id = "odom"
        self.t.child_frame_id = "base_footprint"

        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(self.update_rate)

    def calculate_steps(self, time):
        """
        Function to calculate the amount of steps for each trayectory.
        """
        return int(time * 25)

    def parameters_callback(self, msg):
        data = msg.doubles

        self.update_rate = data[0].value
        self.simulation_factor = data[1].value
        self.rate = rospy.Rate(self.update_rate)

    def cob_tf_compute(self):
        while True:
            self.current_x_position = pose1.position.x
            self.current_y_position = pose1.position.y
            self.current_z_orientation = pose1.orientation.z

            for i in range(0, len(poses_list) - 1):
                diff_x_position = (
                    poses_list[i + 1][0].position.x - poses_list[i][0].position.x
                )

                diff_y_position = (
                    poses_list[i + 1][0].position.y - poses_list[i][0].position.y
                )

                diff_z_orientation = (
                    poses_list[i + 1][0].orientation.z - poses_list[i][0].orientation.z
                )

                while (
                    (
                        abs(poses_list[i + 1][0].position.x - self.current_x_position)
                        > 0.1
                    )
                    and (
                        abs(poses_list[i + 1][0].position.y - self.current_y_position)
                        > 0.1
                    )
                    and (
                        abs(
                            poses_list[i + 1][0].orientation.z
                            - self.current_z_orientation
                        )
                        > 0.1
                    )
                ):
                    for j in range(0, poses_list[i][1]):

                        self.current_x_position += diff_x_position
                        self.current_y_position += diff_y_position
                        self.current_z_orientation += diff_z_orientation

            rospy.sleep()

    def cob_tf_publisher(self):
        while True:
            self.t.header.stamp = rospy.Time.now()
            self.t.transform.translation.x = self.current_x_position
            self.t.transform.translation.y = self.current_y_position
            self.t.transform.translation.z = 0

            self.t.transform.rotation.x = tf.transformations.quaternion_from_euler(
                0, 0, self.current_z_orientation
            )[0]
            self.t.transform.rotation.y = tf.transformations.quaternion_from_euler(
                0, 0, self.current_z_orientation
            )[1]
            self.t.transform.rotation.z = tf.transformations.quaternion_from_euler(
                0, 0, self.current_z_orientation
            )[2]
            self.t.transform.rotation.w = tf.transformations.quaternion_from_euler(
                0, 0, self.current_z_orientation
            )[3]

            self.br.sendTransformMessage(self.t)

            self.rate.sleep()


if __name__ == "__main__":

    cob_transform_publisher = COB_TF_Publisher()
