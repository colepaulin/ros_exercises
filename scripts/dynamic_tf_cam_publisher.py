#!/usr/bin/env python

import rospy
import tf2_ros as tf
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft

rospy.init_node("dynamic_tf_cam_publisher", anonymous = True)
tf_broadcaster = tf.TransformBroadcaster()

tf_buffer = tf.Buffer()
tf_listener = tf.TransformListener(tf_buffer)

rate = rospy.Rate(10)

def pose_from_transform(trans):
    rotation = trans.transform.rotation
    translation = trans.transform.translation
    pose = tft.quaternion_matrix([rotation.x,rotation.y,rotation.z,rotation.w])
    pose[0,3] = translation.x
    pose[1,3] = translation.y
    pose[2,3] = translation.z
    pose[3,3] = 1
    return pose

while not rospy.is_shutdown():
    try:
        world_to_base_tf = tf_buffer.lookup_transform("world", "base_link_gt", rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rate.sleep()
        continue

    bot_pose = pose_from_transform(world_to_base_tf)

    LeftCam_wrt_base = np.array([[1,0,0,-.05],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    LeftCam_wrt_world = np.dot(bot_pose,LeftCam_wrt_base)

    RightCam_wrt_LeftCam = np.array([[1,0,0,.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    left_cam_tf = TransformStamped()

    left_cam_tf.header.stamp = rospy.Time.now()
    left_cam_tf.header.frame_id = "world"
    left_cam_tf.child_frame_id = "left_cam"

    (left_cam_tf.transform.translation.x, left_cam_tf.transform.translation.y, left_cam_tf.transform.translation.z) = (LeftCam_wrt_world[0,3],LeftCam_wrt_world[1,3],LeftCam_wrt_world[2,3])

    (left_cam_tf.transform.rotation.x,left_cam_tf.transform.rotation.y,left_cam_tf.transform.rotation.z,left_cam_tf.transform.rotation.w) = tft.quaternion_from_matrix(LeftCam_wrt_world)

    right_cam_tf = TransformStamped()

    right_cam_tf.header.stamp = rospy.Time.now()
    right_cam_tf.header.frame_id = "left_cam"
    right_cam_tf.child_frame_id = "right_cam"

    (right_cam_tf.transform.translation.x, right_cam_tf.transform.translation.y, right_cam_tf.transform.translation.z) = (RightCam_wrt_LeftCam[0,3],RightCam_wrt_LeftCam[1,3],RightCam_wrt_LeftCam[2,3])

    (right_cam_tf.transform.rotation.x, right_cam_tf.transform.rotation.y, right_cam_tf.transform.rotation.z,right_cam_tf.transform.rotation.w) = tft.quaternion_from_matrix(RightCam_wrt_LeftCam)

    tf_broadcaster.sendTransform(left_cam_tf)
    tf_broadcaster.sendTransform(right_cam_tf)

    rate.sleep()






