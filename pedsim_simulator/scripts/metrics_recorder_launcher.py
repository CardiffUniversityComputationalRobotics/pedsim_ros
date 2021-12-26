import time
import roslaunch
import rospy

PROCESS_GENERATE_RUNNING = True


class ProcessListener(roslaunch.pmon.ProcessListener):
    """keeps track of the process from launch file"""

    global PROCESS_GENERATE_RUNNING

    def process_died(self, name, exit_code):
        global PROCESS_GENERATE_RUNNING
        PROCESS_GENERATE_RUNNING = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
    """initiates launch file runs it"""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile],
        process_listeners=[process_listener],
    )
    return launch


rospy.init_node("async_cnn_generator")
LAUNCH_FILE = "/home/sasm/people_sim_ws/src/pedsim_ros/pedsim_simulator/launch/cob_pedsim_pedestrians_rviz_metrics_test.launch"
launch = init_launch(LAUNCH_FILE, ProcessListener())
launch.start()

init_time = time.time()

while time.time() - init_time < 30:
    rospy.sleep(0.05)

launch.shutdown()
