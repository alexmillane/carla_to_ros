
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

# Carla conversion stuff
from carla import image_converter

# Importing carla conversions
from conversions import *
from common import *

class MessageBuilder(object):
    """Node example class."""

    def __init__(self, K, p_B_C):
        # A cv _bridge for conversions
        self._bridge = CvBridge()
        # The camera intrinsic matrix
        self._image_width = image_width
        self._image_height = image_height
        self._image_fov = image_fov
        self._K = K
        self._K_inv = np.linalg.inv(self._K)
        # # The camera positions
        # # TODO(alex.millane): The constants should be passed around as arguments
        self._p_B_C = p_B_C

    def _get_timestamp_message(self, measurements_carla):
        ms_to_s = 1/1000.0
        return rospy.Time(measurements_carla.game_timestamp * ms_to_s)

    def _get_mono_image_message(self, data_carla, timestamp):
        # Extracts the numpy array and then converts it to a rosmessage
        # print "rgb size: " + str(data_carla.data.shape)
        mono_img = cv2.cvtColor(data_carla.data, cv2.COLOR_RGB2GRAY)
        img_msg = self._bridge.cv2_to_imgmsg(mono_img, "mono8")
        img_msg.header.stamp = timestamp
        return img_msg

    def _get_color_image_message(self, data_carla, timestamp):
        # Extracts the numpy array and then converts it to a rosmessage
        img_msg = self._bridge.cv2_to_imgmsg(data_carla.data, "rgb8")
        img_msg.header.stamp = timestamp
        return img_msg

    def _get_metric_depth_message(self, carla_depth_image, timestamp):
        # Extracts the numpy array and then converts it to a rosmessage
        #data_ic = image_converter.depth_to_logarithmic_grayscale(carla_depth_image)
        far_plane_distance = 1000.0
        metric_depth = image_converter.depth_to_array(
            carla_depth_image) * far_plane_distance
        metric_depth_msg = self._bridge.cv2_to_imgmsg(metric_depth.astype(np.float32)) #, "rgb16"
        metric_depth_msg.header.stamp = timestamp
        return metric_depth_msg

    def _get_channeled_depth_message(self, carla_depth_image):
        print "carla_depth_image.dtype: " + str(carla_depth_image.data.dtype)
        print "NOT YET IMPLEMENTED"

    def _get_transform_message(self, measurements_carla, timestamp):
        # Getting the transform out of the carla data
        transform = measurements_carla.player_measurements.transform
        # Converting carla convensions to ROS convensions as numpy arrays
        position = carla_position_to_position(transform.location)
        orientation = carla_rotator_to_quaternion(transform.rotation)
        # Building the message
        transform_msg = TransformStamped()
        # Need to replace this with true timestamp
        transform_msg.header.stamp = timestamp
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

    def _get_rotator_message(self, measurements_carla, timestamp):
        # Getting the transform out of the carla data
        transform = measurements_carla.player_measurements.transform
        # Building the message
        rotator_msg = Vector3Stamped()
        # Need to replace this with true timestamp
        rotator_msg.header.stamp = timestamp
        rotator_msg.vector.x = transform.rotation.pitch
        rotator_msg.vector.y = transform.rotation.roll
        rotator_msg.vector.z = transform.rotation.yaw
        return rotator_msg

    def _get_pointcloud_message_from_carla_data(self, carla_depth_image, carla_rgb_image, timestamp):
        # Converting to numpy arrays
        depth_array = image_converter.depth_to_array(carla_depth_image)
        bgra_array = image_converter.to_bgra_array(carla_rgb_image)
        # Converting to metric depth
        far_plane_distance = 1000.0
        metric_depth_array = depth_array * far_plane_distance
        # Getting the pointcloud message
        return self._get_pointcloud_msg(self, metric_depth_array, bgra_array, timestamp)

    def _get_pointcloud_message(self, depth_array, bgra_array, timestamp):
        # Getting the pointcloud as a vector of 3D points
        pointcloud_vec = self._get_pointcloud_vector(
            depth_array, bgra_array)
        # Filling out the pointcloud message
        # NOTE(alexmillane): Standard create cloud function works but is slow as fuck.
        #                    I have therefore create a record array and get the bytes directly
        #pointcloud = pc2.create_cloud(header, fields, pointcloud_vec)
        header = Header()
        header.stamp = timestamp
        header.frame_id = "cam"
        fields = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                  pc2.PointField('rgba', 12, pc2.PointField.UINT32, 1)]
        pointcloud = PointCloud2(header=header,
                                 height=1,
                                 width=len(pointcloud_vec),
                                 is_dense=True,
                                 is_bigendian=False,
                                 fields=fields,
                                 point_step=16,
                                 row_step=16 * len(pointcloud_vec),
                                 data=pointcloud_vec.tostring())
        # Creating and returning the pointcloud
        return pointcloud

    def _get_pointcloud_vector(self, metric_depth_array, bgra_array):
        # Taken and modified from:
        # https://github.com/carla-simulator/carla/pull/100/files
        # The length of a vector of all the pixels in the image
        pixel_length = self._image_width * self._image_height
        # Extracts the metric depth values to a flat vector
        metric_depth_vec = np.reshape(metric_depth_array, pixel_length)
        # Gets the image space coordinates as two flat vectors
        u_coord = repmat(np.r_[self._image_width-1:-1:-1],
                         self._image_height, 1).reshape(pixel_length)
        v_coord = repmat(np.c_[self._image_height-1:-1:-1],
                         1, self._image_width).reshape(pixel_length)
        # Homogeneous coordinates of pixels in a flattened vector ( pd2 = [u,v,1] )
        p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])
        # Back-projecting to 3D ( p3d = [X,Y,Z] )
        p3d = np.dot(self._K_inv, p2d)
        #p3d *= metric_depth_vec
        p3d = (p3d * metric_depth_vec).astype(np.dtype('float32'))
        # Getting the color image as a 1 array with uint8s packed into a uint32
        bgra_image = bgra_array.astype(np.uint32)
        rgba_vec_packed = np.left_shift(bgra_image[:, :, 3], 24) + \
            np.left_shift(bgra_image[:, :, 2], 16) + \
            np.left_shift(bgra_image[:, :, 1], 8) + \
            np.left_shift(bgra_image[:, :, 0], 0)
        rgba_vec_packed = np.reshape(rgba_vec_packed, [pixel_length, 1])
        # Packing into a record array (an array with differing types)
        # NOTE(alexmillane): random negatives here for LHC shit
        x_col, y_col, z_col = np.split(p3d.transpose(), 3, axis=1)
        pointcloud_rec_vec = np.rec.fromarrays(
            (-x_col, -y_col, z_col, rgba_vec_packed))
        # Return
        return pointcloud_rec_vec

    def _get_static_transform_message(self, timestamp):
        # Building the message
        transform_msg = TransformStamped()
        # Need to replace this with true timestamp
        transform_msg.header.stamp = timestamp
        transform_msg.child_frame_id = "cam"
        transform_msg.header.frame_id = "car"
        cm_to_m = 1.0 / 100.0
        transform_msg.transform.translation.x = self._p_B_C[0] * cm_to_m
        transform_msg.transform.translation.y = self._p_B_C[1] * cm_to_m
        transform_msg.transform.translation.z = self._p_B_C[2] * cm_to_m
        # Rotating from camera x forward, z up to z forward, x left, y up (retarded convension)
        #quaternion = tf.transformations.quaternion_from_euler(0.0, np.pi/2.0, np.pi/2, axes='rxyz')
        # Rotating from camera x forward, z up, to z forward, x right, y down 
        quaternion = tf.transformations.quaternion_from_euler(0.0, np.pi/2.0, -np.pi/2, axes='rxyz')
        #print "static quaternion: " + str(quaternion)
        transform_msg.transform.rotation.x = quaternion[0]
        transform_msg.transform.rotation.y = quaternion[1]
        transform_msg.transform.rotation.z = quaternion[2]
        transform_msg.transform.rotation.w = quaternion[3]
        return transform_msg


