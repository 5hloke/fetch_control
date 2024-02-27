#!/usr/bin/env python

import sys
import actionlib
import control_msgs.msg
# import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
# import moveit_commander
import rospy
# # import tf
import numpy as np

# from .arm_joints import ArmJoints
# from .moveit_goal_builder import MoveItGoalBuilder
# from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction
# from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
# from tf.listener import TransformListener

ARM_GROUP_NAME = 'arm'
# #Adi: Need to use arm and torso for linear joint trajectories
JOINT_ACTION_SERVER = 'arm_controller/follow_joint_trajectory'
# JOINT_ACTION_SERVER = 'arm_with_torso_controller/follow_joint_trajectory'
TIME_FROM_START = 5


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._joint_client = actionlib.SimpleActionClient(
            JOINT_ACTION_SERVER, control_msgs.msg.FollowJointTrajectoryAction)
        self._joint_client.wait_for_server(rospy.Duration(10))
        # self._tf_listener = TransformListener()

    def move_to_waypoints(self, waypoints):
        trajectory = JointTrajectory()
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        for i in range(waypoints.shape[0]):
            point = JointTrajectoryPoint()
            point.positions = waypoints[i, 1, :]
            point.velocity = waypoints[i, 2, :]
            point.accelerations = [0.0] * len(waypoints[i, 1, :]) # Because there is no acceleration data
            point.time_from_start = rospy.Duration(waypoints[i, 0, 0])
            trajectory.points.append(point)
        goal.trajectory = trajectory
        goal.time_tolerance = rospy.Duration(0.0)
        self._joint_client.send_goal(goal)
    
    def get_client(self):
        return self._joint_client



#     def move_to_joints(self, joint_state):
#         goal = control_msgs.msg.FollowJointTrajectoryGoal()
#         goal.trajectory.joint_names.extend(ArmJoints.names())
#         point = trajectory_msgs.msg.JointTrajectoryPoint()
#         point.positions.extend(joint_state.values())
#         point.time_from_start = rospy.Duration(TIME_FROM_START)
#         goal.trajectory.points.append(point)
#         self._joint_client.send_goal(goal)
#         self._joint_client.wait_for_result(rospy.Duration(10))

#     def cancel_all_goals(self):
#         self._joint_client.cancel_all_goals()

if __name__ == "__main__" :
    joint_States = np.load("../export/output_traj.npy")
    wrist_roll_traj = np.zeros((joint_States.shape[0], 3, 1))
    joint_States = np.concatenate((joint_States, wrist_roll_traj), axis=2)
    print(joint_States.shape)
    print(joint_States[1])
    # joint = ArmJoints()
    # rospy.init_node("prepare_simulated_robot")

    # Check robot serial number, we never want to run this on a real robot!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
        rospy.logerr("This script should not be run on a real robot")
        sys.exit(-1)


    rospy.loginfo("Waiting for arm_controller...")
    arm = Arm()
    rospy.loginfo("...connected.")

    rospy.loginfo("Setting positions...")
    arm.move_to_waypoints(joint_States)
    arm.get.client.wait_for_result(rospy.Duration(joint_States[-1, 0, 0]))
    # head_client.wait_for_result(rospy.Duration(6.0))
    rospy.loginfo("...done")
    
