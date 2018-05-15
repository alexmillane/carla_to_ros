#!/usr/bin/env python

import time
from os.path import expanduser

# Numpy and OpenCV
import cv2
import numpy as np
from numpy.matlib import repmat

# ROS stuff.
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from tf.msg import tfMessage
from cv_bridge import CvBridge
import tf
import rosbag


# Client wide types and constants
from common import *
from message_builder import MessageBuilder


class CarlaBagCompleter(object):
    """Node example class."""

    def __init__(self):
        # CV bidge for image conversions
        self._bridge = CvBridge()
        # Opening bags to read and write from
        home_path = expanduser("~")
        in_bag_name = "carla.bag"
        out_bag_name = "carla_complete.bag"
        in_bag_path = home_path + "/trunk/datasets/carla/" + in_bag_name
        out_bag_path = home_path + "/trunk/datasets/carla/" + out_bag_name
        print in_bag_path
        print out_bag_path
        self._in_bag = rosbag.Bag(in_bag_path, 'r')
        self._out_bag = rosbag.Bag(out_bag_path, 'w')
        # The camera intrinsic matrix
        # TODO(alex.millane): The constants should be passed around as arguments
        self._image_width = image_width
        self._image_height = image_height
        self._image_fov = image_fov
        self._K = self._get_camera_intrinsic_matrix()
        # The camera positions
        # TODO(alex.millane): The constants should be passed around as arguments
        self._p_B_C = np.array(
            [camera_position_x, camera_position_y, camera_position_z])
        # Creating a message builder
        self._message_builder = MessageBuilder(self._K, self._p_B_C)

    def execute(self):

        # Topics to read
        topics = ['image', 'depth', 'tf', 'transform']

        # Image vector
        img_vec = []
        depth_vec = []

        # Looping over the bag and reading
        msg_idx = 0
        msg_num = self._in_bag.get_message_count(topics)
        for topic, msg, t in self._in_bag.read_messages(topics=topics):
            # Debug progress
            print "processed " + str(msg_idx) + "/" + str(msg_num) + " msgs"
            msg_idx += 1

            if topic == 'image':
                img_vec.append(msg)
                self._out_bag.write(topic, msg, msg.header.stamp)

            if topic == 'depth':
                depth_vec.append(msg)
                self._out_bag.write(topic, msg, msg.header.stamp)

            if topic == 'transform':
                self._out_bag.write(topic, msg, msg.header.stamp)

            if topic == 'tf':
                self._out_bag.write(topic, msg, msg.transforms[0].header.stamp)

        # Searching for timestamp matches and computing poinclouds
        msg_idx = 0
        msg_num = len(img_vec)
        for image_msg in img_vec:
            # Debug progress
            print "processed " + str(msg_idx) + "/" + str(msg_num) + " msgs"
            msg_idx += 1
            img_stamp = image_msg.header.stamp
            for depth_msg in depth_vec:
                depth_stamp = depth_msg.header.stamp
                # Matching rgb and depth, make a pointcloud
                if img_stamp == depth_stamp:
                    pointlcoud_msg = self._create_pointcloud_message(
                        image_msg, depth_msg, img_stamp)
                    self._out_bag.write(
                        'pointcloud', pointlcoud_msg, image_msg.header.stamp)
                else:
                    pass

        # Closing the bags properly
        self._in_bag.close()
        self._out_bag.close()

    def _get_camera_intrinsic_matrix(self):
        # (Intrinsic) K Matrix
        K = np.identity(3)
        K[0, 2] = self._image_width / 2.0
        K[1, 2] = self._image_height / 2.0
        K[0, 0] = K[1, 1] = self._image_width / \
            (2.0 * np.tan(self._image_fov * np.pi / 360.0))
        # Printing for use outside
        print "K: " + str(K)
        return K

    def _create_pointcloud_message(self, image_msg, depth_msg, timestamp):

        # Getting the opencv2 image from
        rgb_image = self._bridge.imgmsg_to_cv2(image_msg)
        metric_depth_image = self._bridge.imgmsg_to_cv2(depth_msg)

        # Adding an alpha channel
        image_bgra = np.zeros(
            [rgb_image.shape[0], rgb_image.shape[1], 4], dtype=np.uint8)
        image_bgra[:, :, 0] = rgb_image[:, :, 2]
        image_bgra[:, :, 1] = rgb_image[:, :, 1]
        image_bgra[:, :, 2] = rgb_image[:, :, 0]
        image_bgra[:, :, 3] = 255 * \
            np.ones([rgb_image.shape[0], rgb_image.shape[1]])

        # Creating the poincloud message
        pointlcoud_msg = self._message_builder._get_pointcloud_message(
            metric_depth_image, image_bgra, timestamp)

        return pointlcoud_msg

        # print "rgb_image.dtype: " + str(rgb_image.dtype)
        # print "rgb_image.shape: " + str(rgb_image.shape)
        # print "metric_depth_image.dtype: " + str(metric_depth_image.dtype)
        # print "metric_depth_image.shape: " + str(metric_depth_image.shape)
        # print "image_rgba.dtype: " + str(image_rgba.dtype)


def main():
    carla_bag_completer = CarlaBagCompleter()
    carla_bag_completer.execute()


# Entry
if __name__ == '__main__':
    # Registering ros node
    rospy.init_node('complete_carla_bag')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        main()
    except rospy.ROSInterruptException:
        pass
