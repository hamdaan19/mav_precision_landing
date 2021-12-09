#!/usr/bin/env python
from inspect import Traceback
import rospy
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import Pose, Point, PointStamped
from utils import TargetTracker, Utils
from land_mav import MakeTrajectory
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import time

class MarkerPose(TargetTracker, Utils, MakeTrajectory):
    def __init__(self, raw_image_topic, disparity_topic):
        super(MarkerPose, self).__init__()
        self.raw_image_topic = raw_image_topic
        self.disparity_topic = disparity_topic
        self.sub1 = rospy.Subscriber(self.raw_image_topic, Image, self.retrieve_frame)
        self.sub2 = rospy.Subscriber(self.disparity_topic, DisparityImage, self.callback_disparity)
        self.sub3 = rospy.Subscriber("/firefly/odometry_sensor1/pose", Pose, self.pose)
        
        self.raw_image = None
        self.disparity_image = None
        self.u_o = 376 # camera center X
        self.v_o = 240 # camera center Y

        self.marker_ID = 35

        self.baseline = 0.11 * 100         # centimeters
        self.focal_length = 0.0028 * 1000  # millimeters
        self.prop_constant = 0.6135        # sensor specific

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_frame = "world"
        #self.camera_frame = "firefly/vi_sensor/vi_sensor_link"
        self.camera_frame = "firefly/base_link"


    def retrieve_frame(self, data):
        self.raw_image = super().ros2cv2(data, encoding="bgr8") 


    def pose(self, data):
        self.current_x = data.position.x
        self.current_y = data.position.y
        self.current_z = data.position.z


    def callback_disparity(self, data):
        self.disparity_image = super().ros2cv2(data.image, encoding="passthrough")

        estimated_depth, u, v = self.depth_estimator()
        if estimated_depth != None: 
            #rospy.loginfo(f"Marker at depth: {estimated_depth}")
            estimated_pose = self.pose_estimator(estimated_depth, u, v)

        super().display_image(["disparity", "raw_image"], [self.disparity_image*0.1, self.raw_image])


    def depth_estimator(self):
        box, id = super().detect_marker(self.raw_image)
        if np.any(id): 
            point = super().find_center(box, id, frame=self.raw_image, draw=True)
        else:
            point = {}

        try: 
            u, v = point[self.marker_ID]
            disparity = self.disparity_image[v,u]
            if disparity != 0:
                depth = (self.baseline * self.focal_length)/(disparity * self.prop_constant)
            return (depth, u, v)
        except KeyError:
            return (None, None, None)

    def pose_estimator(self, depth, u, v):
        x = (u-self.u_o) * depth / (self.focal_length)
        y = (v-self.v_o) * depth / (self.focal_length)
        z = -depth

        self.find_transformation(x, y, z)

        

    
    def find_transformation(self, x, y, z):
        try:
            transformation = self.tf_buffer.lookup_transform(self.map_frame, self.camera_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from {0} to {1}'.format(self.camera_frame, self.map_frame))
        point_wrt_camera_frame = Point(x, y, z)

        self.X, self.Y, self.Z = self.transform_point(transformation, point_wrt_camera_frame)       
        self.start_landing()

        #rospy.loginfo(f"Transformed Point: {self.X}\t{self.Y}\t{self.Z}")

    def transform_point(self, transformation, point_wrt_camera_frame):
        point_wrt_map_frame = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=point_wrt_camera_frame), 
            transformation
            ).point
        
        return [point_wrt_map_frame.x, point_wrt_map_frame.y, point_wrt_map_frame.z]


    def start_landing(self):
        if self.current_z-self.Z < 0.1:
            self.stop_flying()
            print("stop flying", self.current_z, self.Z)
        else:
            current_point = np.asarray([self.current_x, self.current_y, self.current_z])
            goal_point = np.asarray([2, 2, self.Z])
            next_point = self.compute_next_point(current_point, goal_point)
            print(current_point, next_point)
            self.publish_next_point(next_point)
            print("keep landing")
        

    def loop(self):
        rospy.logwarn("Entering into the loop...")
        rospy.spin()



