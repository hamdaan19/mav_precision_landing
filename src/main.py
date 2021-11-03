#!/usr/bin/env python
from inspect import Traceback
import cv2
import numpy as np
import rospy
from marker_pose_estimator import MarkerPose
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped



if __name__ == '__main__':
    rospy.init_node('markerDetector', anonymous=False)

    raw_image_topic = "/firefly/vi_sensor/left/image_raw"
    disparity_topic = "/my_stereo/disparity" 

    test = MarkerPose(raw_image_topic=raw_image_topic, disparity_topic=disparity_topic)
    test.loop()