#!/usr/bin/env python

# I will creat a package out of this later

from arm import Arm 
from torso import Torso
import numpy as np
import rospy
import sys
if __name__ == "__main__":
    rospy.init_node("prepare_simulated_robot")
    joint_states = np.load("../export_0304/1_folded_to_straight/output_traj.npy")
    wrist_roll_traj = np.zeros((joint_states.shape[0], 3, 1))
    joint_States = np.concatenate((joint_states, wrist_roll_traj), axis=2)
    
    # Check robot serial number, uncomment this line when running on fetch!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX": 
        rospy.logerr("This script should not be run on a real robot")
        sys.exit(-1)
    
    rospy.loginfo("Waiting for toso controller...")
    torso = Torso()
    rospy.loginfo("...connected.")
    rospy.loginfo("Setting torso...")
    torso.set_height(0.4)
    rospy.loginfo("...done")

    rospy.loginfo("Waiting for arm_controller...")
    arm = Arm()
    rospy.loginfo("...connected.")

    rospy.loginfo("Setting positions...")
    arm.move_to_waypoints(joint_States)
    arm.wait_client(joint_States[-1, 0, 0] + 5)
    rospy.loginfo("...done")
    

