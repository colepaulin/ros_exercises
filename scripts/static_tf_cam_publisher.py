#!/usr/bin/env python

import rospy
import tf2_ros as tf2
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft

rospy.init_node("static_tf_cam_publisher")
static_broadcast = tf2.StaticTransformBroadcaster()

left_cam_wrt_base = np.array([[1,0,0,-.05],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
right_cam_wrt_base = np.array([[1, 0, 0, .05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

left_quat_from_matrix = tft.quaternion_from_matrix(left_cam_wrt_base)
right_quat_from_matrix = tft.quaternion_from_matrix(right_cam_wrt_base)

right_cam_tf = TransformStamped()
left_cam_tf = TransformStamped()

left_cam_tf.header.stamp = rospy.Time.now()
left_cam_tf.header.frame_id = "base_link_gt"
left_cam_tf.child_frame_id = "left_cam"

right_cam_tf.header.stamp = rospy.Time.now()
right_cam_tf.header.frame_id = "base_link_gt"
right_cam_tf.child_frame_id = "right_cam"

left_cam_tf.transform.translation.x = left_cam_wrt_base[0,3]
left_cam_tf.transform.translation.y = left_cam_wrt_base[1, 3]
left_cam_tf.transform.translation.z = left_cam_wrt_base[2, 3]

right_cam_tf.transform.translation.x = right_cam_wrt_base[0, 3]
right_cam_tf.transform.translation.y = right_cam_wrt_base[1, 3]
right_cam_tf.transform.translation.z = right_cam_wrt_base[2, 3]

left_cam_tf.transform.rotation.x = left_quat_from_matrix[0]
left_cam_tf.transform.rotation.y = left_quat_from_matrix[1]
left_cam_tf.transform.rotation.z = left_quat_from_matrix[2]
left_cam_tf.transform.rotation.w = left_quat_from_matrix[3]

right_cam_tf.transform.rotation.x = right_quat_from_matrix[0]
right_cam_tf.transform.rotation.y = right_quat_from_matrix[1]
right_cam_tf.transform.rotation.z = right_quat_from_matrix[2]
right_cam_tf.transform.rotation.w = right_quat_from_matrix[3]

static_broadcast.sendTransform([left_cam_tf,right_cam_tf])
rospy.spin()
