#!/usr/bin/env python3

# general packages
from math import nan
from threading import current_thread
import time
import numpy as np
import csv
import os

import math
import re
from rosgraph_msgs.msg import Clock
from rospy.core import traceback
import rostopic
import rospkg
from datetime import datetime
import rosparam
import yaml

# ros packages
import rospy
from std_msgs.msg import Int16,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from pedsim_msgs.msg import AgentStates ##TODO add pedsim position

# for transformations
from tf.transformations import euler_from_quaternion


class DataCollector:
    def __init__(self, topic,ns):
        topic_callbacks = [
            (f"{ns}scan", self.laserscan_callback),
            (f"{ns}odom", self.odometry_callback),
            (f"{ns}cmd_vel", self.action_callback),
            (f"{ns}pedsim_simulator/simulated_agents", self.human_callback),
            (f"{ns}global_plan", self.callback_plan)
        ]
 
        try:

            callback = lambda msg: [t[1] for t in topic_callbacks if t[0] == topic[0]][0](msg)

        except:
            traceback.print_exc()
            return
        self.full_topic_name = topic[1]
        self.data = None
        self.subscriber = rospy.Subscriber(topic[0], topic[2], callback)

    def callback_plan(self, plan_msg):
        data = []
        for poses in plan_msg.poses:
            data.append([round(poses.pose.position.x,3) ,round(poses.pose.position.y,3)]) 

        self.data = data


    def episode_callback(self, msg_scenario_reset):
        print(msg_scenario_reset)
        
        self.data = msg_scenario_reset.data

    def laserscan_callback(self, msg_laserscan: LaserScan):
        self.data = [msg_laserscan.range_max if math.isnan(val) else round(val, 3) for val in msg_laserscan.ranges]

    def odometry_callback(self, msg_odometry: Odometry):
        pose3d = msg_odometry.pose.pose
        twist = msg_odometry.twist.twist
        self.data = {
            "position": [
                round(val, 3) for val in [
                    pose3d.position.x,
                    pose3d.position.y,
                    euler_from_quaternion(
                        [
                            pose3d.orientation.x, 
                            pose3d.orientation.y,
                            pose3d.orientation.z,
                            pose3d.orientation.w
                        ]
                    )[2]
                ]
            ],
            "velocity": [
                round(val, 3) for val in [
                    twist.linear.x,
                    twist.linear.y,
                    twist.angular.z
                ]
            ]
        }
        # self.start = self.data["position"][0]

    def action_callback(self, msg_action: Twist): # variables will be written to csv whenever an action is published
        self.data = [
            round(val, 3) for val in [
                msg_action.linear.x,
                msg_action.linear.y,
                msg_action.angular.z
            ]
        ]

    def human_callback(self,msg_: AgentStates):
        
        data = []

        for status in msg_.agent_states:
            data.append([ round(status.pose.position.x,3) ,round(status.pose.position.y,3)])
        
        self.data = data
    
    def get_data(self):
        return (
            self.full_topic_name,
            self.data 
        )

        
class Recorder:
    def __init__(self):
        self.take_sencario = os.path.join(rospy.get_param('/scenario_path'),rospy.get_param('/scenario_name')+".json")
        self.model = rospy.get_param("/model")
        self.dir = rospkg.RosPack().get_path("evaluation")
        self.result_dir = os.path.join(self.dir, "data", datetime.now().strftime("%d-%m-%Y_%H-%M-%S")) + "_" + rospy.get_namespace().replace("/", "")+ "_"+ rospy.get_param("/training_model")+"_pomdp_"+str(rospy.get_param("/is_pomdp"))
        try:
            os.mkdir(self.result_dir)
        except:
            pass
        
        self.write_params()
        ns = rospy.get_namespace()
        topics_to_monitor = self.get_topics_to_monitor(ns)

        topics = rostopic.get_topic_list()
        published_topics = topics[0]
        topic_matcher = re.compile(f"({'|'.join([t[0] for t in topics_to_monitor])})$")
        topics_to_sub = []
        for t in published_topics:
            topic_name = t[0]
            match = re.search(topic_matcher, topic_name)

            if  match == None: 
                continue
            if topic_name =="/test_1/scenario_reset":
                continue

            topics_to_sub.append([topic_name, *self.get_class_for_topic_name(topic_name,ns)])


        self.data_collectors = []
        for topic in topics_to_sub:
            self.data_collectors.append(DataCollector(topic,ns))
            self.write_data(
                topic[1], [
                    "time", "data"
                ],
                mode="w"
            )

        self.write_data("episode", ["time", "episode"], mode="w")
        self.write_data("start_goal", ["episode", "start" ,"goal"], mode="w")

        self.current_episode = 0
        self.past_reset = 0 
        self.config = self.read_config()

        self.clock_sub = rospy.Subscriber(f"{ns}clock", Clock, self.clock_callback)
        self.scenario_reset_sub = rospy.Subscriber("/scenario_reset", Int16, self.scenario_reset_callback)
        self.sub_shutdown = rospy.Subscriber("/crowd/record_shutdown", Bool, self.cb_sub_shutdown)
        self.current_time = None
        self.odom = []

    def cb_sub_shutdown(self,msg):
        if msg.data == True:
            rospy.signal_shutdown("Everything is done")
        else:
            pass
     
    def scenario_reset_callback(self, data: Int16):
        self.current_episode = data.data
        self.odom = []

    def clock_callback(self, clock: Clock):
        current_simulation_action_time = clock.clock.secs * 10e9 + clock.clock.nsecs

        if not self.current_time:
            self.current_time = current_simulation_action_time

        time_diff = (current_simulation_action_time - self.current_time) / 1e6 ## in ms

        if time_diff < self.config["record_frequency"]:
            return

        self.current_time = current_simulation_action_time
        
        for collector in self.data_collectors:
                topic_name, data = collector.get_data()

                if topic_name == "odom":
                    self.odom.append(data["position"])
                    if len(self.odom)>1:
                        self.odom.pop(-1)
                    
                    if self.odom[0] == [0.0,0.0,0.0]:
                        self.odom.pop(0)
                if len(self.odom) == 1:
                    self.write_data(topic_name, [self.current_time, data])

        if len(self.odom)==1:
            self.write_data("start_goal", [
            self.current_episode, 
            rospy.get_param(rospy.get_namespace() + "start"),
            rospy.get_param(rospy.get_namespace() + "goal")
            ])
            self.write_data("episode", [self.current_time, self.current_episode])

    def read_config(self):
        with open(self.dir + "/data_recorder_config.yaml") as file:
            return yaml.safe_load(file)

    def get_class_for_topic_name(self, topic_name, ns):
        rospy.logwarn(f"[topic_name]:{topic_name}")
        if f"{ns}scan" in topic_name:
            return ["scan", LaserScan]

        if f"{ns}odom" in topic_name:
            return ["odom", Odometry]

        if f"{ns}cmd_vel" in topic_name:
            return ["cmd_vel", Twist]

        if f"{ns}pedsim_simulator/simulated_agents" in topic_name:
            return ["human",AgentStates]
        
        if f"{ns}global_plan" in topic_name:
            return ["global_plan",Path]
        # else:
        #     return ["scan", LaserScan]
    def get_topics_to_monitor(self,ns):
        return [
            (f"{ns}scan", LaserScan),
            (f"{ns}scenario_reset", Int16),
            (f"{ns}odom", Odometry),
            (f"{ns}cmd_vel", Twist),
            (f"{ns}pedsim_simulator/simulated_agents",AgentStates),
            (f"{ns}global_plan",Path)
        ]

    def write_data(self, file_name, data, mode="a"):
        # if file_name == "human":
        #     pass
        with open(f"{self.result_dir}/{file_name}.csv", mode, newline = "") as file:
            writer = csv.writer(file, delimiter = ',')
            writer.writerow(data)
            file.close()
    
    def write_params(self):
        print(self.take_sencario)
        with open(self.result_dir + "/params.yaml", "w") as file:
            yaml.dump({
                "model": self.model,
                "map_file": rospy.get_param(f"{rospy.get_namespace()}map_file", ""),
                "scenario_file": f"{self.take_sencario.split('/')[-2]}/{self.take_sencario.split('/')[-1]}",
                "local_planner": rospy.get_param('/training_model'),
                "agent_name": rospy.get_param(rospy.get_namespace() + "agent_name", ""),
                "namespace": rospy.get_namespace().replace("/", "")
            }, file)


if __name__=="__main__":
    rospy.init_node("data_recorder", anonymous=True) 

    time.sleep(5)   

    Recorder()

    rospy.spin()

