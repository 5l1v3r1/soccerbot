#!/usr/bin/env python
import rospy
import time
import math
from VelCmd import VelCmd
from GetModelState import *
def talker():
    diffx, diffy = getposition()
    angle = math.atan(diffy,diffx)
    targetangle = w+angle/2/pi*1
    pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = VelCmd()
    msg.Vx = 0 #diffx*10
    msg.Vy = diffy*10 #looks like y doesnt do anything
    msg.w = 1
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
    	#time.sleep(1)
def testrotate(): 
    diffx, diffy, w = getposition()
    pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = VelCmd()
    msg.Vx = 0 #diffx*10
    msg.Vy = 1 #looks like y doesnt do anything
    msg.w = 0.5
    while 1:
        diffx, diffy, w = getposition()
        rospy.loginfo(msg)
        pub.publish(msg)
        print(w)
    	#time.sleep(1)
def rotator():
    diffx, diffy,w = getposition()
    angle = math.atan(diffy/diffx)

    targetangle = w+angle/2/math.pi*1
    pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = VelCmd()
    msg.Vx = 0 #diffx*10
    msg.Vy = 1 #looks like y doesnt do anything
    msg.w = 0.1
    while abs(w-targetangle)>0.05:
        diffx, diffy, w = getposition()
        rospy.loginfo(msg)
        pub.publish(msg)
        print(w-targetangle)
    	#time.sleep(1)
    print(w, targetangle)

def getposition():
    rospy.wait_for_service('/gazebo/get_model_state') 
    getmodelstate = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    n1output = getmodelstate('nubot1','chassis')
    foutput = getmodelstate('football','chassis')
    print(n1output.pose.orientation.w)
    diffx = foutput.pose.position.x-n1output.pose.position.x
    diffy = foutput.pose.position.y-n1output.pose.position.y
    w = n1output.pose.orientation.w
    return diffx, diffy, abs(w)
    #info = rospy.ServiceProxy('/gazebo/get_model_state', getmodelstate)

if __name__ == '__main__':
    try:
        testrotate()
    except rospy.ROSInterruptException: pass
