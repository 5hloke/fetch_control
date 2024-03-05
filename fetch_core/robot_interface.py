import controller_manager_msgs.srv
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf import TransformListener, TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import copy, math, rospy, time
#import config_core as cfg # ?
import numpy as np

from arm import Arm
from arm_joints import ArmJoints
from base import Base
from camera import RGBD
from head import Head
from gripper import Gripper
from torso import Torso
from reader import JointStateReader
DEG_TO_RAD = np.pi / 180
ZRANGE = 20


class Robot_Interface(object):
    """For usage with the Fetch robot."""

    def __init__(self, simulation=True):
        """Initializes various aspects of the Fetch.
        
        TODOs: get things working, also use `simulation` flag to change ROS
        topic names if necessary (especially for the cameras!). UPDATE: actually
        I don't think this is necessary now, they have the same topic names.
        """
        rospy.init_node("fetch")
        self.arm = Arm()
        self.arm_joints = ArmJoints()
        self.base = Base()
        self.camera = RGBD()
        self.head = Head()
        self.gripper = Gripper(self.camera)
        self.torso = Torso()
        self.joint_reader = JointStateReader()

        # Tucked arm starting joint angle configuration
        self.names = ArmJoints().names()
        self.tucked = [1.3200, 1.3999, -0.1998, 1.7199, 0.0, 1.6600, 0.0]
        self.tucked_list = [(x,y) for (x,y) in zip(self.names, self.tucked)]

        # Initial (x,y,yaw) position of the robot wrt map origin. We keep this
        # fixed so that we can reset to this position as needed. The HSR's
        # `omni_base.pose` (i.e., the start pose) returns (x,y,yaw) where yaw is
        # the rotation about that axis (intuitively, the z axis). For the base,
        # `base.odom` supplies both `position` and `orientation` attributes.
        start = copy.deepcopy(self.base.odom.position)
        yaw = Base._yaw_from_quaternion(self.base.odom.orientation)
        self.start_pose = np.array([start.x, start.y, yaw])
        self.TURN_SPEED = 0.3

        self.num_restarts = 0


    def get_img_data(self):
        """Obtain camera and depth image.
        
        Returns:
            Tuple containing RGB camera image and corresponding depth image.
        """
        c_img = self.camera.read_color_data()
        d_img = self.camera.read_depth_data()
        return (c_img, d_img)


    def get_depth(self, point, d_img):
        """Compute mean depth near grasp point.

        NOTE: assumes that we have a simlar `cfg.ZRANGE` as with the HSR. I'm
        not sure where exactly this comes from.
        """
        y, x = int(point[0]), int(point[1])
        z_box = d_img[y-ZRANGE:y+ZRANGE, x-ZRANGE:x+ZRANGE]
        indx = np.nonzero(z_box)
        z = np.mean(z_box[indx])
        return z