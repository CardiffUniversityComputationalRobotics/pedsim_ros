#!/usr/bin/env python3

import rospy
from pedsim_msgs.msg import FrozenAgents
import csv
from datetime import datetime


def import_csv(csvfilename):
    data = []
    with open(csvfilename, "r", encoding="utf-8", errors="ignore") as scraped:
        reader = csv.reader(scraped, delimiter=",")
        row_index = 0
        for row in reader:
            if row:  # avoid blank lines
                row_index += 1
                columns = [str(row_index), row[0], row[1], row[2]]
                data.append(columns)
        scraped.close()
    return data


class FrozenAgentCounter:
    """This class manages the state of the agents based on it position and time"""

    def save_value_csv(self):
        # TODO: create or modify csv adding value
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

        last_data = None
        try:
            csv_read_data = import_csv(
                self.csv_dir + self.solution_type + "/" + self.csv_name
            )
            last_data = csv_read_data[-1]
        except:
            pass

        with open(
            self.csv_dir + self.solution_type + "/" + self.csv_name, "a", newline=""
        ) as csvfile_write, open(
            self.csv_dir + self.solution_type + "/" + self.csv_name,
            "r",
        ) as csvfile_read:
            reader = csv.reader(csvfile_read)
            fieldnames = ["test_number", "time", "frozen_counter"]
            writer = csv.DictWriter(csvfile_write, fieldnames=fieldnames)
            try:
                if next(reader) != ["test_number", "time", "frozen_counter"]:
                    writer.writeheader()
            except:
                writer.writeheader()

            if last_data is not None:
                writer.writerow(
                    {
                        "test_number": int(last_data[1]) + 1,
                        "time": dt_string,
                        "frozen_counter": self.frozen_agents_counter,
                    }
                )
            else:
                writer.writerow(
                    {
                        "test_number": 1,
                        "time": dt_string,
                        "frozen_counter": self.frozen_agents_counter,
                    }
                )
            print("[INFO] [" + str(rospy.get_time()) + "] Frozen Agents Counter saved.")
            csvfile_write.close()
            csvfile_read.close()

    def __init__(self):

        rospy.init_node("frozen_agent_csv_writer", anonymous=True)

        rospy.on_shutdown(self.save_value_csv)

        # dict to know if agent was frozen before
        self.agents_register_dict = {}

        # frozen agents counter
        self.frozen_agents_counter = 0

        # ! directory of csv files
        self.csv_dir = rospy.get_param(
            "/frozen_agent_csv/csv_dir",
            "/home/sasm/people_sim_ws/src/pedsim_ros/pedsim_simulator/metrics/",
        )

        self.solution_type = rospy.get_param(
            "/frozen_agent_csv/solution_type", "position/"
        )

        self.csv_name = rospy.get_param("/frozen_agent_csv/csv_name", "first_test.csv")

        # subcribers
        self._agents_status_sub = rospy.Subscriber(
            "/frozen_agents",
            FrozenAgents,
            self.agent_freezing_callback,
            queue_size=1,
        )

    def agent_freezing_callback(self, data):
        """AgentStates subscribe to get agents position"""
        agents_status = data.frozen_agents
        for agent in agents_status:
            if str(agent.id) in self.agents_register_dict:
                if (
                    agent.is_frozen == "stuck"
                    and self.agents_register_dict[str(agent.id)] == "moving"
                ):
                    self.frozen_agents_counter += 1
                    self.agents_register_dict[str(agent.id)] == "stuck"
                elif agent.is_frozen == "moving":
                    self.agents_register_dict[str(agent.id)] == "moving"
            else:
                self.agents_register_dict[str(agent.id)] = "moving"


if __name__ == "__main__":
    csv_counter_saver = FrozenAgentCounter()
    while True:
        rospy.spin()
