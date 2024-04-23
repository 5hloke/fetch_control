#!/usr/bin/env python

# I will creat a package out of this later

from arm import Arm 
from torso import Torso
from robot_interface import Fetch
import numpy as np
import rospy
import sys
import matplotlib.pyplot as plt

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
    print(fetch.arm.feedback[0])
    t_stamp = np.arange(len(fetch.arm.feedback))
    des_position = np.zeros((len(fetch.arm.feedback), 7))
    des_velocity = np.zeros((len(fetch.arm.feedback), 7))

    actual_positon = np.zeros((len(fetch.arm.feedback), 7))
    actual_velocity = np.zeros((len(fetch.arm.feedback), 7))

    error_position = np.zeros((len(fetch.arm.feedback), 7))
    error_velocity = np.zeros((len(fetch.arm.feedback), 7))
    # print(fetch.arm.feedback)
    for i in range(len(fetch.arm.feedback)):
        des_position[i] = np.array(fetch.arm.feedback[i].desired.positions)
        des_velocity[i] = np.array(fetch.arm.feedback[i].desired.velocities)

        actual_positon[i] = np.array(fetch.arm.feedback[i].actual.positions)
        actual_velocity[i] = np.array(fetch.arm.feedback[i].actual.velocities)

        error_position[i] = np.array(fetch.arm.feedback[i].error.positions)
        error_velocity[i] = np.array(fetch.arm.feedback[i].error.velocities)
    
    plt.plot(des_position[:, 0], label="desired")
    plt.plot(actual_positon[:, 0],label="actual")
    plt.plot(error_position[:, 0], label="error")
    plt.xlabel("time")
    plt.ylabel("position")
    plt.legend()
    plt.savefig("position1.png")

    plt.plot(des_position[:, 1], label="desired")
    plt.plot(actual_positon[:, 1],label="actual")
    plt.plot(error_position[:, 1], label="error")
    plt.xlabel("time")
    plt.ylabel("position")
    plt.legend()
    plt.savefig("position2.png")

    plt.plot(des_position[:, 2], label="desired")
    plt.plot(actual_positon[:, 2],label="actual")
    plt.plot(error_position[:, 2], label="error")
    plt.xlabel("time")
    plt.ylabel("position")
    plt.legend()
    plt.savefig("position3.png")

    plt.plot(des_velocity[:, 0], label="desired")
    plt.plot(actual_velocity[:, 0],label="actual")
    plt.plot(error_velocity[:, 0], label="error")
    plt.xlabel("time")
    plt.ylabel("velocity")
    plt.legend()
    plt.savefig("velocity1.png")

    plt.plot(des_velocity[:, 1], label="desired")
    plt.plot(actual_velocity[:, 1],label="actual")
    plt.plot(error_velocity[:, 1], label="error")
    plt.xlabel("time")
    plt.ylabel("velocity")
    plt.legend()
    plt.savefig("velocity2.png")

    plt.plot(des_velocity[:, 2], label="desired")
    plt.plot(actual_velocity[:, 2],label="actual")
    plt.plot(error_velocity[:, 2], label="error")
    plt.xlabel("time")
    plt.ylabel("velocity")
    plt.legend()
    plt.savefig("velocity3.png")



    

    # rospy.loginfo("...connected.")

    # rospy.loginfo("Setting positions...")
    # arm.move_to_waypoints(joint_States)
    # arm.wait_client(joint_States[-1, 0, 0] + 5)
    # rospy.loginfo("...done")
    

