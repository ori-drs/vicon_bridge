#!/usr/bin/env python
import rospy
from vicon_bridge.msg import Markers
from visualization_msgs.msg import Marker


def callback(in_msg):

    out_msg = Marker()

    out_msg.header = in_msg.header
    out_msg.header.frame_id = 'vicon/world'
    out_msg.ns = 'anymal'
    out_msg.action = out_msg.ADD
    out_msg.type = out_msg.POINTS
    out_msg.id = 0
    out_msg.scale.x = 0.02
    out_msg.scale.y = 0.02
    out_msg.scale.z = 0.02

    out_msg.color.a = 1.0
    out_msg.color.r = 1.0
    out_msg.color.g = 0.5
    out_msg.color.b = 0.5

    rospy.loginfo("converting message..")
    out_msg.points=[]
    for marker in in_msg.markers:
        if marker.segment_name=='anymal':
            out_msg.points.append(marker.translation)
            out_msg.points[-1].x = out_msg.points[-1].x/1000
            out_msg.points[-1].y = out_msg.points[-1].y/1000
            out_msg.points[-1].z = out_msg.points[-1].z/1000

    global pub
    pub.publish(out_msg)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('vicon_marker_convertor')

    rospy.Subscriber("/vicon/markers", Markers, callback)
    global pub
    pub = rospy.Publisher('/vicon/markers/marker_array', Marker, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

# This node subscribes to a vicon_bridge/Markers topic
# and republishes the data as a visualization_msgs/MarkerArray for
# visualisation in RVIZ.
