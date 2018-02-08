#!/usr/bin/env python

import argparse
import logging
import random
import time


# Numpy and OpenCV
import cv2
import numpy as np

# ROS stuff.
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf

# Importing the python client to the simulator
#from carla.client import *
from carla.client import make_carla_client
from carla.tcp import TCPConnectionError
from carla import image_converter # Only for debug

# Importing the interactive game
from carla_interaction import CarlaGame

class CarlaToRos(object):
    """Node example class."""

    def __init__(self):
        # Create a publisher for data messages.
        self.image_pub = rospy.Publisher('image', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('depth', Image, queue_size=10)
        # A cv bridge for conversions
        self.bridge = CvBridge()
        # A tf broadcaster for 
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Get parameters
        #self.enable = rospy.get_param('~enable', True)
        #self.int_a = rospy.get_param('~a', 1)
        #self.int_b = rospy.get_param('~b', 2)
        #self.message = rospy.get_param('~message', 'hello')

        # if self.enable:
        #     self.start()
        # else:
        #     self.stop()

    def data_callback(self, measurements, sensor_data):
        print "In data_callback"
        # Publishing the image message over ros
        image_msg = self._get_image_msg(sensor_data['CameraRGB'])
        self.image_pub.publish(image_msg)
        # Publishing the depth
        depth_msg = self._get_depth_msg(sensor_data['CameraDepth'])
        self.depth_pub.publish(depth_msg)

        # Inspecting the measurements
        tf_msg = self._get_transform_msg(measurements)

    def _get_image_msg(self, data_carla):
        # Extracts the numpy array and then converts it to a rosmessage
        #print "rgb size: " + str(data_carla.data.shape)
        return self.bridge.cv2_to_imgmsg(data_carla.data, "rgb8")

    def _get_depth_msg(self, data_carla):
        # Extracts the numpy array and then converts it to a rosmessage
        #data_np = data_carla.data
        #print "depth size: " + str(data_carla.data.shape)
        #print "max: " + str(np.max(data_np))
        #print "min: " + str(np.min(data_np))

        # Note(alexmillane):
        #  data_carla.data: np array with depth from 0.0 - 0.1 as 32bit float
        #  logarithmic greyscale: np array with 0.0 - 255.0 as 32bit float
        #  output: image with 3 channels with uint16 scaled to fit maximums

        data_ic = image_converter.depth_to_logarithmic_grayscale(data_carla)
        #data_ic = data_ic * (pow(2,8)-1)
        #print "max_ic: " + str(np.max(data_ic))
        #print "min_ic: " + str(np.min(data_ic))

        #data_np_16 = (data_carla.data * (pow(2,16)-1)).astype(np.uint16)
        # print "max16: " + str(np.max(data_np_16))
        # print "min16: " + str(np.min(data_np_16))

        # return self.bridge.cv2_to_imgmsg(data_np_16, "mono16")

        return self.bridge.cv2_to_imgmsg((data_ic * (pow(2,8)-1)).astype(np.uint16), "rgb16")


    def _get_transform_msg(self, measurements_carla):
        # Printing 
        transform = measurements_carla.player_measurements.transform
        # Converting to quaternion
        transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll
        q = tf.transformations.quaternion_from_euler(transform.rotation.pitch,
                                                     transform.rotation.yaw,
                                                     transform.rotation.roll,
                                                     'sxyz')
        # UP TO HERE. ATTEMPT TO FIX THIS MESS.
        print q
        # Message fields
        translation_tuple = (transform.location.x, transform.location.y, transform.location.z)
        orientation_tuple = ()

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
                carla_to_ros = CarlaToRos()
                game = CarlaGame(client, args.map_name,
                                 data_callback=carla_to_ros.data_callback)
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
