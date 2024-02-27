import rospy
import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import actionlib
from tf.listener import TransformListener
import math

from .arm_joints import ArmJoints

ARM_GROUP_NAME = 'arm'
JOINT_ACTION_SERVER = 'arm_with_torso_controller/follow_joint_trajectory'

class ArmControl(object):
    
    def __init__(self):
        self._joint_client = actionlib.SimpleActionClient(
            JOINT_ACTION_SERVER, control_msgs.action.FollowJointTrajectory)
        self._joint_client.wait_for_server()
        self._tf = TransformListener()
    
    def cancel_all_goals(self):
        self._joint_client.cancel_all_goals()
    
    def move_to_joints(self):
        goal = control_msgs.action.FollowJointTrajectory()
        goal.trajectory.joint_names = 

        
