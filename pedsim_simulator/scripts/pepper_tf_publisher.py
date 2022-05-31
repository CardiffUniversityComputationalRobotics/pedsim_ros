#!/usr/bin/env python3

from time import sleep
import roslib
import rospy
import tf
from geometry_msgs.msg import Pose, TransformStamped
from dynamic_reconfigure.msg import Config

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


def calculate_steps(time):
    """
    Function to calculate the amount of steps for each trayectory.
    """
    return int(time * 100)


poses_list = [
    [pose1, calculate_steps(20)],  # recta
    [pose2, calculate_steps(4)],  # giro
    [pose3, calculate_steps(8)],  # recta
    [pose4, calculate_steps(5)],  # giro
    [pose5, calculate_steps(15)],  # recta
    [pose6, calculate_steps(8)],  # giro regreso
    [pose7, calculate_steps(15)],  # recta
    [pose8, calculate_steps(4)],  # giro
    [pose9, calculate_steps(9)],  # recto
    [pose10, calculate_steps(3)],  # giro
    [pose11, calculate_steps(20)],  # recta
    [pose12, calculate_steps(5)],  # giro
    [pose13, calculate_steps(5)],  # no se da tiempo
]


class CobTfPublisher:
    """
    Publishes the Pose of the robot as fast as the simulation factor specifies.
    """

    def __init__(self):
        rospy.init_node("simulator_tf_broadcaster")
        self.rqt_parameters_sub = rospy.Subscriber(
            "/pedsim_simulator/parameter_updates",
            Config,
            self.parameters_callback,
            queue_size=1,
        )
        self.update_rate = 100
        self.simulation_factor = 1

        self.current_x_position = pose1.position.x
        self.current_y_position = pose1.position.y
        self.current_z_orientation = pose1.orientation.z

        self._t = TransformStamped()
        self._t.header.frame_id = "odom"
        self._t.child_frame_id = "base_footprint"

        self._br = tf.TransformBroadcaster()

    def parameters_callback(self, msg):
        """
        updates simulation factor and update rate modified by rqt_reconfigure
        """
        data = msg.doubles
        # self.update_rate = data[0].value
        self.simulation_factor = data[1].value
        # self.rate = rospy.Rate(self.update_rate * self.simualtion_factor)
        # print("Rate changed for transform")

    def cob_tf_publisher(self):
        """
        Publishes and changes the Pose of the cob robot.
        """
        while not rospy.is_shutdown():
            self.current_x_position = pose1.position.x
            self.current_y_position = pose1.position.y
            self.current_z_orientation = pose1.orientation.z

            for i in range(0, len(poses_list) - 1):
                diff_x_position = (
                    poses_list[i + 1][0].position.x - poses_list[i][0].position.x
                ) / poses_list[i][1]

                diff_y_position = (
                    poses_list[i + 1][0].position.y - poses_list[i][0].position.y
                ) / poses_list[i][1]

                diff_z_orientation = (
                    poses_list[i + 1][0].orientation.z - poses_list[i][0].orientation.z
                ) / poses_list[i][1]

                for j in range(0, poses_list[i][1]):

                    self._t.header.stamp = rospy.Time.now()
                    self._t.transform.translation.x = self.current_x_position
                    self._t.transform.translation.y = self.current_y_position
                    self._t.transform.translation.z = 0

                    self._t.transform.rotation.x = (
                        tf.transformations.quaternion_from_euler(
                            0, 0, self.current_z_orientation
                        )[0]
                    )
                    self._t.transform.rotation.y = (
                        tf.transformations.quaternion_from_euler(
                            0, 0, self.current_z_orientation
                        )[1]
                    )
                    self._t.transform.rotation.z = (
                        tf.transformations.quaternion_from_euler(
                            0, 0, self.current_z_orientation
                        )[2]
                    )
                    self._t.transform.rotation.w = (
                        tf.transformations.quaternion_from_euler(
                            0, 0, self.current_z_orientation
                        )[3]
                    )

                    self._br.sendTransformMessage(self._t)

                    self.current_x_position += diff_x_position
                    self.current_y_position += diff_y_position
                    self.current_z_orientation += diff_z_orientation

                    sleep(1 / (self.update_rate * self.simulation_factor))
                    rospy.sleep(-1)


if __name__ == "__main__":

    cob_transform_publisher = CobTfPublisher()
    cob_transform_publisher.cob_tf_publisher()
