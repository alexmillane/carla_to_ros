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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from tf.msg import tfMessage
from cv_bridge import CvBridge
import tf
import rosbag

# For debugging the euler angles
from geometry_msgs.msg import Vector3Stamped

# Importing the python client to the simulator
#from carla.client import *
from carla.client import make_carla_client
from carla.tcp import TCPConnectionError
from carla import image_converter  # Only for debug

# Importing the interactive game
from carla_interaction import CarlaGame

# Client wide types and constants
from common import *


class CarlaToRos(object):
    """Node example class."""

    def __init__(self, game):
        # Create a publisher for data messages.
        self._image_pub = rospy.Publisher('image', Image, queue_size=10)
        self._depth_pub = rospy.Publisher('depth', Image, queue_size=10)
        # A cv _bridge for conversions
        self._bridge = CvBridge()
        # A tf broadcaster for
        self._tf_broadcaster = tf.TransformBroadcaster()
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
        self._K_inv = np.linalg.inv(self._K)
        # The camera positions
        # TODO(alex.millane): The constants should be passed around as arguments
        self._camera_position_x = camera_position_x
        self._camera_position_y = camera_position_y
        self._camera_position_z = camera_position_z

    def data_callback(self, measurements, sensor_data):
        print "In data_callback"

        # Shutting down if requested
        if rospy.is_shutdown():
            print "Shutting down the interface."
            self.shutdown()
            return

        # Getting the msgs from Carla data
        image_msg = self._get_image_msg(sensor_data['CameraRGB'])
        depth_msg = self._get_depth_msg(sensor_data['CameraDepth'])
        transform_msg = self._get_transform_message(measurements)
        static_transform_msg = self._get_static_transform_message()
        #rotator_msg = self._get_rotator_message(measurements)

        # Zeroing the position to be relative to the first frame
        remove_position_offset = True
        if remove_position_offset is True:
            transform_msg = self._remove_position_offset(transform_msg)

        # Making the tf message
        tf_msg = self._get_tf_message([transform_msg, static_transform_msg])

        # Getting the pointcloud
        pointcloud_start = time.time()
        pointcloud_msg = None
        pointcloud_msg = self._get_pointcloud_msg(
           sensor_data['CameraDepth'], sensor_data['CameraRGB'])
        pointcloud_end = time.time()
        print(pointcloud_end - pointcloud_start)

        # TODO: SHOULD BE PASSING THIS FUNCTION THE MESSAGE
        # self._publish_tf(measurements)

        # Publishing the transform
        publish_data = False
        if publish_data is True:
            self._publish_data(image_msg, depth_msg)

        # Writing data to a bag
        write_data_to_bag = True
        if write_data_to_bag is True:
            self._write_data_to_bag(
                image_msg, depth_msg, transform_msg, tf_msg, pointcloud_msg)

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
        return K

    def _get_image_msg(self, data_carla):
        # Extracts the numpy array and then converts it to a rosmessage
        # print "rgb size: " + str(data_carla.data.shape)
        return self._bridge.cv2_to_imgmsg(data_carla.data, "rgb8")

    def _get_depth_msg(self, data_carla):
        # Extracts the numpy array and then converts it to a rosmessage
        data_ic = image_converter.depth_to_logarithmic_grayscale(data_carla)
        return self._bridge.cv2_to_imgmsg((data_ic * (pow(2, 8) - 1)).astype(np.uint16), "rgb16")

    def _get_transform_message(self, measurements_carla):
        # Getting the transform out of the carla data
        transform = measurements_carla.player_measurements.transform
        # Converting carla convensions to ROS convensions as numpy arrays
        position = self._carla_position_to_position(transform.location)
        orientation = self._carla_rotator_to_quaternion(transform.rotation)
        # Building the message
        transform_msg = TransformStamped()
        # Need to replace this with true timestamp
        transform_msg.header.stamp = rospy.get_rostime()
        transform_msg.child_frame_id = "car"
        transform_msg.header.frame_id = "world"
        transform_msg.transform.translation.x = position[0]
        transform_msg.transform.translation.y = position[1]
        transform_msg.transform.translation.z = position[2]
        transform_msg.transform.rotation.x = orientation[0]
        transform_msg.transform.rotation.y = orientation[1]
        transform_msg.transform.rotation.z = orientation[2]
        transform_msg.transform.rotation.w = orientation[3]
        return transform_msg

    def _get_tf_message(self, transform_msgs):
        # Constructing the message
        tf_msg = tfMessage(transform_msgs)
        # Returning the message
        return tf_msg

    def _get_rotator_message(self, measurements_carla):
        # Getting the transform out of the carla data
        transform = measurements_carla.player_measurements.transform
        # Building the message
        rotator_msg = Vector3Stamped()
        # Need to replace this with true timestamp
        rotator_msg.header.stamp = rospy.get_rostime()
        rotator_msg.vector.x = transform.rotation.pitch
        rotator_msg.vector.y = transform.rotation.roll
        rotator_msg.vector.z = transform.rotation.yaw
        return rotator_msg

    def _get_pointcloud_msg(self, carla_depth_image, carla_rgb_image):
        # Getting the pointcloud as a vector of 3D points
        projecting_start_time = time.time()
        pointcloud_vec = self._get_pointcloud_vector(
            carla_depth_image, carla_rgb_image)
        projecting_end_time = time.time()
        print "projecting time: " + str(projecting_end_time - projecting_start_time)
        # Filling out the pointcloud message
        encoding_start_time = time.time()
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = "cam"
        # fields = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        #           pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        #           pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1)]
        fields = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('rgba', 12, pc2.PointField.UINT32, 1)]
                  # pc2.PointField('r', 12, pc2.PointField.UINT8, 1),
                  # pc2.PointField('g', 13, pc2.PointField.UINT8, 1),
                  # pc2.PointField('b', 14, pc2.PointField.UINT8, 1)]
        pointcloud = pc2.create_cloud(header, fields, pointcloud_vec)
        encoding_end_time = time.time()
        print "encoding time: " + str(encoding_end_time - encoding_start_time)
        # Creating and returning the pointcloud
        return pointcloud

    def _get_pointcloud_vector(self, carla_depth_image, carla_rgb_image):
        # Taken and modified from:
        # https://github.com/carla-simulator/carla/pull/100/files
        # The length of a vector of all the pixels in the image
        pixel_length = self._image_width * self._image_height
        # Extracts the metric depth values to a flat vector
        far_plane_distance = 100.0
        metric_depth_vec = image_converter.depth_to_array(
            carla_depth_image) * far_plane_distance
        metric_depth_vec = np.reshape(metric_depth_vec, pixel_length)
        # Gets the image space coordinates as two flat vectors
        u_coord = repmat(np.r_[self._image_width-1:-1:-1],
                         self._image_height, 1).reshape(pixel_length)
        v_coord = repmat(np.c_[self._image_height-1:-1:-1],
                         1, self._image_width).reshape(pixel_length)
        # Homogeneous coordinates of pixels in a flattened vector ( pd2 = [u,v,1] )
        p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])
        # Back-projecting to 3D ( p3d = [X,Y,Z] )
        p3d = np.dot(self._K_inv, p2d)
        p3d *= metric_depth_vec
        # Dealing with the color image
        bgra_image = image_converter.to_bgra_array(carla_rgb_image).astype(np.uint32)
        bgra_vec_packed = np.left_shift(bgra_image[:,:,3],24) + \
                           np.left_shift(bgra_image[:,:,2],16) + \
                           np.left_shift(bgra_image[:,:,1],8) + \
                           np.left_shift(bgra_image[:,:,0],0)
        bgra_vec_packed = np.reshape(bgra_vec_packed, [pixel_length,1])
        # Combining the vectors
        pointcloud_vec = np.concatenate((p3d.transpose(), bgra_vec_packed), axis=1)
        # Returning
        return pointcloud_vec

    # def _publish_tf(self, measurements_carla):
    #     # Printing
    #     transform = measurements_carla.player_measurements.transform
    #     # Converting to quaternion
    #     transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll
    #     q = tf.transformations.quaternion_from_euler(transform.rotation.pitch,
    #                                                  transform.rotation.yaw,
    #                                                  transform.rotation.roll,
    #                                                  'sxyz')
    #     # Message fields
    #     translation_tuple = (transform.location.x,
    #                          transform.location.y, transform.location.z)
    #     # Quaternion order is (qx qy qz qw)
    #     orientation_tuple = (q[0], q[1], q[2], q[3])
    #     stamp = rospy.get_rostime()  # Need to replace this with true timestamp
    #     child_frame_id = "car"
    #     parent_frame_id = "world"
    #     # Publish the tf
    #     self._tf_broadcaster.sendTransform(
    #         translation_tuple, orientation_tuple, stamp, child_frame_id, parent_frame_id)

    def _get_static_transform_message(self):
        # Building the message
        transform_msg = TransformStamped()
        # Need to replace this with true timestamp
        transform_msg.header.stamp = rospy.get_rostime()
        transform_msg.child_frame_id = "cam"
        transform_msg.header.frame_id = "car"
        cm_to_m = 1.0 / 100.0
        transform_msg.transform.translation.x = self._camera_position_x * cm_to_m
        transform_msg.transform.translation.y = self._camera_position_y * cm_to_m
        transform_msg.transform.translation.z = self._camera_position_z * cm_to_m
        transform_msg.transform.rotation.x = 0
        transform_msg.transform.rotation.y = 0
        transform_msg.transform.rotation.z = 0
        transform_msg.transform.rotation.w = 1
        return transform_msg

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

    def _carla_position_to_position(self, carla_location):
        # Negating one component because of LHC
        carla_to_metric = 1.0 / 100.0
        return np.array([carla_location.x, -carla_location.y, carla_location.z]) * carla_to_metric

    def _carla_rotator_to_quaternion(self, carla_rotator):
        # Excerpt from unreal math
        # FMath::SinCos(&SP, &CP, Pitch*DIVIDE_BY_2);
        # FMath::SinCos(&SY, &CY, Yaw*DIVIDE_BY_2);
        # FMath::SinCos(&SR, &CR, Roll*DIVIDE_BY_2);

        # FQuat RotationQuat;
        # RotationQuat.X =  CR*SP*SY - SR*CP*CY;
        # RotationQuat.Y = -CR*SP*CY - SR*CP*SY;
        # RotationQuat.Z =  CR*CP*SY - SR*SP*CY;
        # RotationQuat.W =  CR*CP*CY + SR*SP*SY;

        # Extracting components, dividing by 2, converting to radians, and half negating due to LHC
        pitch = (carla_rotator.pitch / 2) * np.pi/180
        roll = -1 * (carla_rotator.roll / 2) * np.pi/180
        yaw = -1 * (carla_rotator.yaw / 2) * np.pi/180
        # Doing the sin and cos
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        # Making the quaternion
        qx = cr*sp*sy - sr*cp*cy
        qy = -cr*sp*cy - sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        qw = cr*cp*cy + sr*sp*sy
        # Creating the vector
        return np.array([qx, qy, qz, qw])

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
