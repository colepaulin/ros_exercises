#!/usr/bin/env python

import rospy
import tf2_ros as tf
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft

rospy.init_node("base_link_tf_pub", anonymous = True)
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
        world_to_leftcam_tf = tf_buffer.lookup_transform("world", "left_cam", rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rate.sleep()
        continue

    world_to_left_pose = pose_from_transform(world_to_leftcam_tf)

    LeftCam_wrt_base = np.array([[1,0,0,-.05],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    base_wrt_leftcam = np.linalg.inv(LeftCam_wrt_base)

    base_wrt_world = np.dot(world_to_left_pose,base_wrt_leftcam)
    base_wrt_world_quaternion = tft.quaternion_from_matrix(base_wrt_world)

    base_wrt_world_tf = TransformStamped()

    base_wrt_world_tf.header.stamp = rospy.Time.now()
    base_wrt_world_tf.header.frame_id = "world"
    base_wrt_world_tf.child_frame_id = "base_link_gt_2"

    base_wrt_world_tf.transform.translation.x = base_wrt_world[0,3]
    base_wrt_world_tf.transform.translation.y = base_wrt_world[1, 3]
    base_wrt_world_tf.transform.translation.z = base_wrt_world[2, 3]

    base_wrt_world_tf.transform.rotation.x = base_wrt_world_quaternion[0]
    base_wrt_world_tf.transform.rotation.y = base_wrt_world_quaternion[1]
    base_wrt_world_tf.transform.rotation.z = base_wrt_world_quaternion[2]
    base_wrt_world_tf.transform.rotation.w = base_wrt_world_quaternion[3]

    tf_broadcaster.sendTransform(base_wrt_world_tf)

    rate.sleep()






