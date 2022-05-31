#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def joint_state_publisher():
    """function to publish continously the position of the joints of care o bot"""
    joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    rospy.init_node("cob_joint_state_publisher")
    rate = rospy.Rate(25)  # 10hz
    joint_state_msg = JointState()
    joint_state_msg.header = Header()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = [
        "fl_caster_rotation_joint",
        "fl_caster_r_wheel_joint",
        "b_caster_rotation_joint",
        "b_caster_r_wheel_joint",
        "fr_caster_rotation_joint",
        "fr_caster_r_wheel_joint",
        "head_1_joint",
        "head_2_joint",
        "head_3_joint",
        "sensorring_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_left_5_joint",
        "arm_left_6_joint",
        "arm_left_7_joint",
        "gripper_right_finger_1_joint",
        "gripper_right_finger_2_joint",
        "gripper_left_finger_1_joint",
        "gripper_left_finger_2_joint",
    ]

    joint_state_msg.position = [
        -9.576824857138178e-05,
        0.10503934348692212,
        3.14159224925643,
        0.10300531224194032,
        9.656550617620496e-05,
        0.10555145456779158,
        1.0072381497749916e-05,
        7.030458642187654e-05,
        -1.8492486283605558e-05,
        0.0006168416326008952,
        2.0874473067861103,
        1.474161799268268,
        -2.9388074593446296,
        -1.4962075806061268,
        0.5245271967529268,
        -1.617664486532144,
        1.8312071828042527,
        -0.8123487476643598,
        -1.4750187323940303,
        -0.2095284824664123,
        -1.4965477701005891,
        0.7510530934233977,
        -1.8370555509091204,
        -1.6082781258138814,
        6.825372034757748e-05,
        -0.0001712272475975496,
        -6.762316934949553e-05,
        -1.9918255839357357e-05,
    ]

    joint_state_msg.velocity = []
    joint_state_msg.effort = []
    while not rospy.is_shutdown():
        joint_states_pub.publish(joint_state_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
