import roslaunch
import rospy
import time

process_generate_running = True


class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile],
        process_listeners=[process_listener],
    )
    return launch


rospy.init_node("async_cnn_generator")
launch_file = "/home/sasm/people_sim_ws/src/pedsim_ros/pedsim_simulator/launch/cob_pedsim_pedestrians_rviz_metrics_test.launch"
launch = init_launch(launch_file, ProcessListener())
launch.start()

init_time = time.time()

while time.time() - init_time < 120:
    rospy.sleep(0.05)

launch.shutdown()
