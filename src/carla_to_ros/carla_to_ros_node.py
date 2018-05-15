#!/usr/bin/env python

import argparse
import logging
import random
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
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from tf.msg import tfMessage
from cv_bridge import CvBridge
import tf
import rosbag

# Importing the python client to the simulator
#from carla.client import *
from carla.client import make_carla_client
from carla.tcp import TCPConnectionError

# Importing the interactive game
from carla_interaction import CarlaGame

# Client wide types and constants
from common import *
from message_builder import MessageBuilder


class CarlaToRos(object):
    """Node example class."""

    def __init__(self, game):
        # Create a publisher for data messages.
        self._image_pub = rospy.Publisher('image', Image, queue_size=10)
        self._depth_pub = rospy.Publisher('depth', Image, queue_size=10)
        # Opening a bag to write to
        home = expanduser("~")
        bag_name = "carla.bag"
        bag_path = home + "/trunk/datasets/carla/" + bag_name
        print bag_path
        self._bag = rosbag.Bag(bag_path, 'w')
        # Storing the game (only so we can exit it)
        self._game = game
        # A flag for the first measurement
        self._first_position_flag = True
        self._first_position = np.array([0.0, 0.0, 0.0])
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

    def data_callback(self, measurements, sensor_data):

        # Time for datacall back timing
        t_start = time.time()

        # Controls for data writing
        record_image = True
        record_depth = True
        record_transform = True
        record_tf = True
        record_pointcloud = False

        # Default is that msgs are None
        image_msg = None
        depth_msg = None
        transform_msg = None
        static_transform_msg = None
        pointcloud_msg = None

        # Shutting down if requested
        if rospy.is_shutdown():
            print "Shutting down the interface."
            self.shutdown()
            return

        # Getting the time stamp
        timestamp = self._message_builder._get_timestamp_message(measurements)

        # Getting the msgs from Carla data
        if record_image is True:
            t_start_image = time.time()
            image_msg = self._message_builder._get_color_image_message(
                sensor_data['CameraRGB'], timestamp)
            t_end_image = time.time()
            # print "image time: " + str(t_end_image - t_start_image)

        if record_depth is True:
            t_start_depth = time.time()
            depth_msg = self._message_builder._get_metric_depth_message(
                sensor_data['CameraDepth'], timestamp)
            # self._get_channeled_depth_msg(sensor_data['CameraDepth'])
            t_end_depth = time.time()
            # print "depth time: " + str(t_end_depth - t_start_depth)

        if record_transform is True:
            t_start_transform = time.time()
            transform_msg = self._message_builder._get_transform_message(
                measurements, timestamp)
            static_transform_msg = self._message_builder._get_static_transform_message(
                timestamp)
            t_end_transform = time.time()
            # print "transform time: " + str(t_end_transform - t_start_transform)

        # Zeroing the position to be relative to the first frame
        remove_position_offset = True
        if remove_position_offset is True:
            transform_msg = self._remove_position_offset(transform_msg)

        # Making the tf message
        if record_tf is True:
            t_start_tf = time.time()
            tf_msg = self._message_builder._get_tf_message(
                [transform_msg, static_transform_msg])
            t_end_tf = time.time()
            # print "tf time: " + str(t_end_tf - t_start_tf)

        # Getting the pointcloud
        if record_pointcloud is True:
            t_start_pointcloud = time.time()
            pointcloud_msg = self._message_builder._get_pointcloud_message_from_carla_data(
                sensor_data['CameraDepth'], sensor_data['CameraRGB'], timestamp)
            t_end_pointcloud = time.time()
            # print "pointcloud time: " + str(t_end_pointcloud - t_start_pointcloud)

        # Publishing the transform
        publish_data = False
        if publish_data is True:
            self._publish_data(image_msg, depth_msg)

        # Writing data to a bag
        write_data_to_bag = True
        if write_data_to_bag is True:
            t_start_bag = time.time()
            self._write_data_to_bag(
                image_msg, depth_msg, transform_msg, tf_msg, pointcloud_msg)
            t_end_bag = time.time()
            # print "bag write time: " + str(t_end_bag - t_start_bag)

        # Time for datacall back timing
        t_end = time.time()
        print "data_callback time: " + str(t_end - t_start)

    def shutdown(self):
        self._game.stop()
        self._bag.close()

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

    def _write_data_to_bag(self, image_msg, depth_msg, transform_msg, tf_msg, pointcloud_msg):
        # Writing that shit
        try:
            if image_msg is not None:
                self._bag.write('image', image_msg)
            if depth_msg is not None:
                self._bag.write('depth', depth_msg)
            if transform_msg is not None:
                self._bag.write('transform', transform_msg)
            if tf_msg is not None:
                self._bag.write('tf', tf_msg)
            if pointcloud_msg is not None:
                self._bag.write('pointcloud', pointcloud_msg)
            #self._bag.write('rotator', rotator_msg)
        except:
            pass

    def _publish_data(self, image_msg, depth_msg):
        # Publishing that shit
        self._image_pub.publish(image_msg)
        self._depth_pub.publish(depth_msg)

    def _remove_position_offset(self, transform_msg):
        # Storing the first position if this is the first message
        if self._first_position_flag is True:
            self._first_position[0] = transform_msg.transform.translation.x
            self._first_position[1] = transform_msg.transform.translation.y
            self._first_position[2] = transform_msg.transform.translation.z
            self._first_position_flag = False
        # Removing the offset from both messages
        transform_msg.transform.translation.x -= self._first_position[0]
        transform_msg.transform.translation.y -= self._first_position[1]
        transform_msg.transform.translation.z -= self._first_position[2]
        # Returning the new transform
        return transform_msg


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-m', '--map-name',
        metavar='M',
        default=None,
        help='plot the map of the current city (needs to match active map in server, options: Town01 or Town02)')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    while True:
        try:

            with make_carla_client(args.host, args.port) as client:
                # Starting the python game
                game = CarlaGame(client, args.map_name)
                # Starting the ros interface and linking it with the game
                carla_to_ros = CarlaToRos(game)
                # Adding the callback to the game
                game.set_data_callback(carla_to_ros.data_callback)
                # Loop and execture the game
                game.execute()
                break

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


# Entry
if __name__ == '__main__':
    # Registering ros node
    rospy.init_node('carla_to_ros')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        main()
    except rospy.ROSInterruptException:
        pass
