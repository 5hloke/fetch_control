#!/usr/bin/env python

# I will creat a package out of this later

from arm import Arm 
from torso import Torso
from robot_interface import Fetch
import numpy as np
import rospy
import sys
if __name__ == "__main__":
    # rospy.init_node("prepare_simulated_robot")
    joint_states = np.load("../export_0304/2_folded_to_side/output_traj.npy")
    wrist_roll_traj = np.zeros((joint_states.shape[0], 3, 1))
    joint_States = np.concatenate((joint_states, wrist_roll_traj), axis=2)
    
    # Check robot serial number, uncomment this line when running on fetch!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX": 
        rospy.logerr("This script should not be run on a real robot")
        sys.exit(-1)
    
    # rospy.loginfo("Waiting for toso controller...")
    # torso = Torso()
    # rospy.loginfo("...connected.")
    # rospy.loginfo("Setting torso...")
    # torso.set_height(0.4)
    # rospy.loginfo("...done")

    rospy.loginfo("Waiting for robot initialization...")
    fetch = Fetch()
    rospy.loginfo("...connected.")

    fetch.read_joint_waypoints(joint_States)

    rospy.loginfo("Setting Torso...")
    fetch.torso.set_height(0.4)
    rospy.loginfo("...done")
    fetch.move_arm_joints(joint_States)
    # print(result)
    rospy.loginfo("...done")
    print(len(fetch.arm.feedback))
    # for i in range(len(fetch.arm.feedback)):
    #     print(fetch.arm.feedback[i])

    # rospy.loginfo("...connected.")

    # rospy.loginfo("Setting positions...")
    # arm.move_to_waypoints(joint_States)
    # arm.wait_client(joint_States[-1, 0, 0] + 5)
    # rospy.loginfo("...done")
    

