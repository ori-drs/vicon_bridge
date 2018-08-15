#!/usr/bin/env python

# Note: TF transformation function APIs can be found here:
# https://github.com/ros/geometry/blob/melodic-devel/tf/src/tf/transformations.py


import rospy
import tf
import numpy

VICON_WORLD_FRAME = 'vicon/world'
VICON_BASE_FRAME = 'vicon/anymal/anymal'
ANYMAL_ODOM_FRAME = 'odom'
ANYMAL_BASE_FRAME = 'base'

def getTransformationMatrix(target, source, time):
    rospy.loginfo("Getting transfrom from %s to %s"%(source,target))

    global tf_listener
    tf_listener.waitForTransform(target, source, time, rospy.Duration(60.0))
    try:
        (trans,rot) = tf_listener.lookupTransform(target, source, time)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Error getting transform..")

    # numpy arrays to 4x4 transform matrix
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    # create a 4x4 matrix
    mat = numpy.dot(trans_mat, rot_mat)
    rospy.loginfo(mat)

    return mat

def init():
    rospy.loginfo("Intialising ros node..")
    rospy.init_node('vicon_offset_publisher')

    global tf_listener
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    time = rospy.Time(0)

    T1 = getTransformationMatrix(VICON_BASE_FRAME , VICON_WORLD_FRAME, time)
    T2 = getTransformationMatrix(ANYMAL_BASE_FRAME, ANYMAL_ODOM_FRAME, time)

    # Calculate transformation to correct offset between vicon and base frames.
    T = numpy.dot(tf.transformations.inverse_matrix(T1),T2)
    q = tf.transformations.quaternion_from_matrix(T)
    t = tf.transformations.translation_from_matrix(T)
    rospy.loginfo(T)
    rospy.loginfo(q)
    rospy.loginfo(t)

    rospy.loginfo("Sending transformation between vicon/world and odom frames...")
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        time = rospy.Time.now()
        tf_broadcaster.sendTransform(t,q,time,ANYMAL_ODOM_FRAME,VICON_WORLD_FRAME)
        #getTransformationMatrix(VICON_BASE_FRAME, ANYMAL_BASE_FRAME,time)
        rate.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
