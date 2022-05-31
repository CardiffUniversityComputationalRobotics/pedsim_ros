#!/bin/zsh
for i in {0..100}
do
   echo "========================================================\n"
   echo "This is the $i th run\n"
   echo "========================================================\n"
   source /home/sasm/people_sim_ws/devel/setup.zsh
   python3 /home/sasm/people_sim_ws/src/pedsim_ros/pedsim_simulator/scripts/metrics_recorder_launcher.py
done
