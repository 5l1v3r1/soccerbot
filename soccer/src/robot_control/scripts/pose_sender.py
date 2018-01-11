import rospy
from std_msgs.msg import String
from geometry_msgs import PoseArray
from geometry_msgs import Point
from geometry_msgs import Quaternion

def talker():
    pub = rospy.Publisher('chatter', PoseArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz = 
    posearray = PoseArray()
    for i in range(20):
	temppoint = Point()
    temppoint.x = i 
    temppoint.y = i 
    temppoint.z = i 
    tempquart = Quaternion()
    tempquart.x = i
    tempquart.y = i 
    tempquart.z = i 
    tempquart.w = i 
    temppose = Pose()
    temppose.position = temppoint
    temppose.orientation = tempquart
    posearray.poses.append(temppose)
    print("This worked")
        
    
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(posearray)
        pub.publish(posearray)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
