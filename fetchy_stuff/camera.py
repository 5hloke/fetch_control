#!/usr/bin/python
# -*- coding: utf-8 -*-
import argparse, cv2, math, os, rospy, sys, threading, time
from pprint import pprint
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

# import tf
# import tf2_ros
# import tf2_geometry_msgs
# import IPython


class RGBD(object):

    def __init__(self):
        """Similar to the HSR version, but with Fetch topic names."""
        topic_name_c = 'head_camera/rgb/image_raw'
        topic_name_i = 'head_camera/rgb/camera_info'
        #Simulation:
        # topic_name_d = 'head_camera/depth_registered/image_raw'
        #Physical Robot
        topic_name_d = 'head_camera/depth/image_raw'

        self._bridge = CvBridge()
        self._input_color_image = None
        self._input_depth_image = None
        self._info = None
        self.is_updated = False

        self._sub_color_image = rospy.Subscriber(topic_name_c, Image, self._color_image_cb)
        self._sub_depth_image = rospy.Subscriber(topic_name_d, Image, self._depth_image_cb)
        self._sub_info        = rospy.Subscriber(topic_name_i, CameraInfo, self._info_cb)


    def _color_image_cb(self, data):
        try:
            self._input_color_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            self.color_time_stamped = data.header.stamp
            self.is_updated = True
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)


    def _depth_image_cb(self, data):
        try:
            #Simulation
            self._input_depth_image = self._bridge.imgmsg_to_cv2(
                    data, desired_encoding="passthrough")
            #Physical Robot
            #self._input_depth_image = self._bridge.imgmsg_to_cv2(
            #        data, "32FC1")

        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)


    def _info_cb(self,data):
        try:
            self._info = data
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)


    def read_color_data(self):
        return self._input_color_image


    def read_depth_data(self):
        return self._input_depth_image


    def read_info_data(self):
        return self._info

# testing the camera interface
if __name__ == "__main__":
    rospy.init_node("fetch_rgbd")
    rgbd = RGBD()
    while not rgbd.is_updated:
        print("Not Updated")
        time.sleep(0.1)
    color_image = rgbd.read_color_data()
    depth_image = rgbd.read_depth_data()
    info = rgbd.read_info_data()
    print("Color image shape: ", color_image.shape)
    print("Depth image shape: ", depth_image.shape)
    print("Camera Info: ", info)
    #save the images to a file
    cv2.imwrite("color_image.png", color_image)
    cv2.imwrite("depth_image.png", depth_image)
    # cv2.imshow('color', color_image)
    # cv2.imshow('depth', depth_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    rospy.spin()
    sys.exit(0)
