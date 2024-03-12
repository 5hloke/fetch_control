#!/usr/bin/env python

import sys
import actionlib
import control_msgs.msg
# import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
# import moveit_commander
import rospy
# import tf
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
arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

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
        self.feedback = []
        # self._tf_listener = TransformListener()

    def move_to_waypoints(self, waypoints):
        trajectory = JointTrajectory()
        trajectory.joint_names = arm_joint_names
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        point = JointTrajectoryPoint()
        for i in range(waypoints.shape[0]):
            trajectory.points.append(point)
            trajectory.points[i].positions = waypoints[i, 1, :]
            trajectory.points[i].velocities = waypoints[i, 2, :]
            trajectory.points[i].accelerations = [0.0] * len(waypoints[i, 1, :]) # Because there is no acceleration data
            trajectory.points[i].time_from_start = rospy.Duration(waypoints[i, 0, 0])
            
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)
        self._joint_client.send_goal(goal, feedback_cb=self.feedback_cb)
     
    def wait_client(self, time):
        self._joint_client.wait_for_result(rospy.Duration(time))
        return self._joint_client.get_result()
    
    def cancel_all_goals(self):
        self._joint_client.cancel_all_goals()
    
    def feedback_cb(self, feedback):
        self.feedback.append(feedback)
        rospy.loginfo("Feedback: {0}".format(feedback))




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

# if __name__ == "__main__" :
#     # joint_States = np.load("../export/output_traj.npy")
#     # wrist_roll_traj = np.zeros((joint_States.shape[0], 3, 1))
#     # joint_States = np.concatenate((joint_States, wrist_roll_traj), axis=2)
#     joint_States = np.array([[[0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
#                               [-3.926990816987241395e-01,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00],
#                               [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00]],
#                               [[5.000000000000000000e-01,5.000000000000000000e-01,5.000000000000000000e-01,5.000000000000000000e-01,5.000000000000000000e-01,5.000000000000000000e-01,0.000000000000000000e+00],
#                                [-3.272493027072578275e-01,-1.820914709666346780e-05,-1.484331590972942649e-08,-5.947842607873384263e-06,0.000000000000000000e+00,5.567689042400634584e-21,0.000000000000000000e+00],
#                                [2.617991159658655254e-01,-7.283658838665378989e-05,-5.937326351523666817e-08,-2.379137043149352011e-05,0.000000000000000000e+00,2.227075616960251125e-20,0.000000000000000000e+00]],
#                               [[1.000000000000000000e+00,1.000000000000000000e+00,1.000000000000000000e+00,1.000000000000000000e+00,1.000000000000000000e+00,1.000000000000000000e+00,0.000000000000000000e+00],
#                                [-1.322088913726780779e-01,-2.803524656443617076e-05,-6.128670904104183137e-10,-6.637936283415404455e-06,0.000000000000000000e+00,7.394671050923295757e-22,0.000000000000000000e+00],
#                                [5.235985116917710736e-01,3.207545874782991881e-05,1.151075934345084931e-07,2.055516832069556610e-05,0.000000000000000000e+00,-4.113822879544370056e-20,0.000000000000000000e+00]],
#                               [[1.500000000000000000e+00,1.500000000000000000e+00,1.500000000000000000e+00,1.500000000000000000e+00,1.500000000000000000e+00,1.500000000000000000e+00,0.000000000000000000e+00],
#                                [1.924221846608391573e-01,-8.653430473523089352e-06,6.079341829945406062e-08,-2.059288671521436032e-06,0.000000000000000000e+00,-7.460715421859675805e-23,0.000000000000000000e+00],
#                                [7.853977626761334818e-01,4.609331479077899516e-05,1.328196992049460678e-07,-1.829474506705760210e-06,0.000000000000000000e+00,3.705916718229100452e-20,0.000000000000000000e+00]],
#                               [[2.000000000000000000e+00,2.000000000000000000e+00,2.000000000000000000e+00,2.000000000000000000e+00,2.000000000000000000e+00,2.000000000000000000e+00,0.000000000000000000e+00],
#                                [5.157445566013625360e-01,-2.091358617652758613e-05,-2.489032135954971636e-08,-4.955701138313943806e-06,0.000000000000000000e+00,5.664202590110151667e-22,0.000000000000000000e+00],
#                                [5.235996803394828980e-01,-9.421207130698153111e-05,-4.728982637030754676e-07,-9.792764850598385413e-06,0.000000000000000000e+00,-3.375387418572679729e-20,0.000000000000000000e+00]],
#                               [[2.500000000000000000e+00,2.500000000000000000e+00,2.500000000000000000e+00,2.500000000000000000e+00,2.500000000000000000e+00,2.500000000000000000e+00,0.000000000000000000e+00],
#                                [7.094771512304699490e-01,-7.367927170373852531e-05,-2.116971531407330076e-08,-1.699633307047381087e-05,0.000000000000000000e+00,9.818943321520214846e-21,0.000000000000000000e+00],
#                                [2.618026917837365519e-01,-1.187349122280019096e-04,4.783227221698974303e-07,-3.856561817505307178e-05,0.000000000000000000e+00,7.008888895204907759e-20,0.000000000000000000e+00]],
#                               [[3.000000000000000000e+00,3.000000000000000000e+00,3.000000000000000000e+00,3.000000000000000000e+00,3.000000000000000000e+00,3.000000000000000000e+00,0.000000000000000000e+00],
#                                [7.736198392127734413e-01,-4.909272899164264211e-05,1.929421085122839941e-07,-1.044756186522611691e-05,0.000000000000000000e+00,5.802720153556556598e-21,0.000000000000000000e+00],
#                                [4.113981151687706728e-06,2.147063848318254780e-04,3.876910276565874019e-07,6.398939063254277873e-05,0.000000000000000000e+00,-8.475200384486288443e-20,0.000000000000000000e+00]],
#                               [[3.500000000000000000e+00,3.500000000000000000e+00,3.500000000000000000e+00,3.500000000000000000e+00,3.500000000000000000e+00,3.500000000000000000e+00,0.000000000000000000e+00],
#                                [7.081726052232739654e-01,-6.819530916112326324e-05,-2.509933150207643848e-08,-1.852479405610070598e-05,0.000000000000000000e+00,1.001653254944946041e-21,0.000000000000000000e+00],
#                                [-2.617929676595263100e-01,-2.868225778131117012e-04,-1.252102966409232965e-06,-9.501853158339038812e-05,0.000000000000000000e+00,6.385269617351896147e-20,0.000000000000000000e+00]],
#                               [[4.000000000000000000e+00,4.000000000000000000e+00,4.000000000000000000e+00,4.000000000000000000e+00,4.000000000000000000e+00,4.000000000000000000e+00,0.000000000000000000e+00],
#                                [5.131361205815614390e-01,-4.203912930612420699e-05,5.927255664062158758e-08,-1.138713936156491057e-05,0.000000000000000000e+00,-3.273373251600093163e-22,0.000000000000000000e+00],
#                                [-5.235888302605142819e-01,3.857108456768464674e-04,1.564548460325852558e-06,1.216687797298655687e-04,0.000000000000000000e+00,-6.789160457046851916e-20,0.000000000000000000e+00]],
#                               [[4.500000000000000000e+00,4.500000000000000000e+00,4.500000000000000000e+00,4.500000000000000000e+00,4.500000000000000000e+00,4.500000000000000000e+00,0.000000000000000000e+00],
#                                [3.194086107341847502e-01,-1.931429270485086799e-05,-1.460354415172560039e-08,-5.301517411710717109e-06,0.000000000000000000e+00,-1.179576986576374582e-21,0.000000000000000000e+00],
#                                [-2.617929857342028988e-01,-2.870972823582160108e-04,-1.828761894031195390e-06,-9.489291633585139390e-05,0.000000000000000000e+00,6.312481383339372389e-20,0.000000000000000000e+00]],
#                               [[5.000000000000000000e+00,5.000000000000000000e+00,5.000000000000000000e+00,5.000000000000000000e+00,5.000000000000000000e+00,5.000000000000000000e+00,0.000000000000000000e+00],
#                                [2.552697019287815117e-01,-6.401823218032631183e-05,-1.637727420344958773e-08,-1.767256939132169595e-05,0.000000000000000000e+00,-3.223782659151351097e-21,0.000000000000000000e+00],
#                                [1.490797905900898002e-06,1.025395788091499304e-04,1.785091735517008211e-06,4.351085009069059826e-05,0.000000000000000000e+00,-7.003914024702571141e-20,0.000000000000000000e+00]]
#                             #   [[5.500000000000000000e+00,5.500000000000000000e+00,5.500000000000000000e+00,5.500000000000000000e+00,5.500000000000000000e+00,5.500000000000000000e+00,0.000000000000000000e+00]],
#                             #   [[6.000000000000000000e+00,6.000000000000000000e+00,6.000000000000000000e+00,6.000000000000000000e+00,6.000000000000000000e+00,6.000000000000000000e+00,0.000000000000000000e+00]],
#                             #   [[6.500000000000000000e+00,6.500000000000000000e+00,6.500000000000000000e+00,6.500000000000000000e+00,6.500000000000000000e+00,6.500000000000000000e+00,0.000000000000000000e+00]],
#                             #   [[7.000000000000000000e+00,7.000000000000000000e+00,7.000000000000000000e+00,7.000000000000000000e+00,7.000000000000000000e+00,7.000000000000000000e+00,0.000000000000000000e+00]],
#                               ])
#     print(joint_States.shape)
#     print(joint_States[1])
#     # np.savetxt('test_time.out', joint_States[:, 0, :], delimiter=',')
#     # joint = ArmJoints()
#     rospy.init_node("prepare_simulated_robot")

#     # Check robot serial number, uncomment this line when running on fetch!
#     if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX": 
#         rospy.logerr("This script should not be run on a real robot")
#         sys.exit(-1)


#     rospy.loginfo("Waiting for arm_controller...")
#     arm = Arm()
#     rospy.loginfo("...connected.")

#     rospy.loginfo("Setting positions...")
#     arm.move_to_waypoints(joint_States)
#     arm.wait_client(joint_States[-1, 0, 0] + 5)
#     rospy.loginfo("...done")
    
