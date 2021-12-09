#!/usr/bin/env python
import rospy
from rospy.topics import Publisher
from geometry_msgs.msg import PoseStamped
import time


if __name__ == '__main__':
    rospy.init_node('Hover', anonymous=False)
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=5)
    dPoint = PoseStamped()
    rate = rospy.Rate(1)
    i = 2
    button = False

    while not rospy.is_shutdown():

        if (i == 10):
            button = True
        elif (i == 2):
            button = False
            
        
        dPoint.header.stamp = rospy.Time.now()
        dPoint.header.frame_id = "world"
        dPoint.pose.orientation.z = 0.0
        dPoint.pose.orientation.w = 0.0

        dPoint.pose.position.x = 2
        dPoint.pose.position.y = 2
        dPoint.pose.position.z = i

        rospy.loginfo(f"Published to topic {i}")
      
        if button == False:
            i += 0.5
        elif button == True:
            i -= 0.5 

        pub.publish(dPoint)
        rate.sleep()

