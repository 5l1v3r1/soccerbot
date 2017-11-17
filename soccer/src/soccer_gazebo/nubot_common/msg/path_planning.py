#!/usr/bin/env python
import rospy
import time
import math
from Pose2D import Pose2D
from Path import Path
def publisher(data):
    node_name = 'path'
    topic_name = 'path_master'

    pub = rospy.Publisher(topic_name, Path)
    rospy.init_node(node_name, Path, anonymous=True) #anonymous for multiple names
    rate = rospy.Rate(10) #10hz
    msg = Path()
    msg.pose.position.x = data.x 
    msg.pose.position.y = data.y
    msg.pose.position.theta= data.theta 
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
    	rate.sleep

def subscriber(): 
    topic_name = 'get_pose2d'
    node_name = 'path_planning' 
    rospy.init_node(node_name, anonymous=True) #anonymous for multiple names
    rospy.Subscriber(subscribe_name, Path, publisher)
    rospy.spin()

if __name__ == '__main__':
    try:
        testrotate()
    except rospy.ROSInterruptException: pass
