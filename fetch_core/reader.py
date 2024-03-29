#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np

class JointStateReader(object):
    """Listens to /joint_states and provides the latest joint angles.

    Usage:
        joint_reader = JointStateReader()
        rospy.sleep(0.1)
        joint_reader.get_joint('shoulder_pan_joint')
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])
    """
    def __init__(self):
        self._sub = rospy.Subscriber('/joint_states', JointState,
            self._callback)
        self._joint_states = {}

    # Global method callback
    def _callback(self, msg):
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            self._joint_states[name] = msg.position[i]

        # rospy.loginfo(self._joint_states)

    def get_joint(self, name):
        """Gets the latest joint value.

        Args:
            name: string, the name of the joint whose value we want to read.

        Returns: the joint value, or None if we do not have a value yet.
        """
        return self._joint_states[name] if name in self._joint_states else None

    def get_joints(self, names):
        """Gets the latest values for a list of joint names.

        Args:
            name: list of strings, the names of the joints whose values we want
                to read.

        Returns: A list of the joint values. Values may be None if we do not
            have a value for that joint yet.
        """
        return [self.get_joint(name) for name in names]
    
# if __name__ == "__main__":
#     rospy.init_node("joint_state_reader")
#     joint_states = np.load("../export_0304/2_folded_to_side/output_traj.npy")
#     joint_reader = JointStateReader()
#     rospy.sleep(0.1)
#     # print(joint_reader.get_joint('shoulder_pan_joint'))
#     # print(joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint']))
#     names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

#     while not rospy.is_shutdown():
#         print(joint_reader.get_joints(names))
#         rospy.sleep(0.5)
#     rospy.spin()