#!/usr/bin/env python

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import math
import moveit_commander
import rospy
import tf

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from tf.listener import TransformListener

ARM_GROUP_NAME = 'arm'
#Adi: Need to use arm and torso for linear joint trajectories
#JOINT_ACTION_SERVER = 'arm_controller/follow_joint_trajectory'
JOINT_ACTION_SERVER = 'arm_with_torso_controller/follow_joint_trajectory'
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
        self._move_group_client = actionlib.SimpleActionClient(
            MOVE_GROUP_ACTION_SERVER, MoveGroupAction)
        self._move_group_client.wait_for_server(rospy.Duration(10))
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._tf_listener = TransformListener()

    def move_to_waypoints(self, waypoints, t):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.extend(['torso_lift_joint'] + ArmJoints.names())
        #goal.trajectory.joint_names.extend(ArmJoints.names())
        for i, w in enumerate(waypoints):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            goal.trajectory.points.append(point)
            goal.trajectory.points[i].time_from_start = rospy.Duration(t[i])
            for j, p in enumerate(waypoints[i]):
                goal.trajectory.points[i].positions.append(waypoints[i][j])
                goal.trajectory.points[i].velocities.append(0.0)
                goal.trajectory.points[i].accelerations.append(0.0)
        self._joint_client.send_goal(goal)
        self._joint_client.wait_for_result(rospy.Duration(10))



    def move_to_joints(self, joint_state):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.extend(ArmJoints.names())
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.extend(joint_state.values())
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        goal.trajectory.points.append(point)
        self._joint_client.send_goal(goal)
        self._joint_client.wait_for_result(rospy.Duration(10))

    def cancel_all_goals(self):
        self._joint_client.cancel_all_goals()
