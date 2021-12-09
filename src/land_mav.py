#!/usr/bin/env python
import rospy
from klampt.model import trajectory
from klampt.io import ros as klampt_ros
import numpy as np
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class MakeTrajectory():
    def __init__(self):
        self.traj_publisher = rospy.Publisher("/firefly/command/trajectory", MultiDOFJointTrajectory, queue_size=5)
        self.point_publisher = rospy.Publisher("/firefly/command/pose", PoseStamped, queue_size=5)
        self.motor_publisher0 = rospy.Publisher("/firefly/motor_speed/0", Float32, queue_size=5)
        self.motor_publisher1 = rospy.Publisher("/firefly/motor_speed/1", Float32, queue_size=5)
        self.motor_publisher2 = rospy.Publisher("/firefly/motor_speed/2", Float32, queue_size=5)
        self.motor_publisher3 = rospy.Publisher("/firefly/motor_speed/3", Float32, queue_size=5)
        self.motor_publisher4 = rospy.Publisher("/firefly/motor_speed/4", Float32, queue_size=5)

    def make_trajectory(self, start_point, goal_point):
        milestones = [start_point, goal_point]
        traj = trajectory.Trajectory(milestones=milestones)
        traj_ = trajectory.HermiteTrajectory()
        traj_.makeSpline(traj)
        traj_timed = trajectory.path_to_trajectory(traj_,vmax=1,amax=1)

        return traj_timed

    def publish_trajectory(self, traj):
        traj_msg = klampt_ros.to_JointTrajectory(traj)
        self.traj_publisher.publish(traj_msg)

    def compute_next_point(self, current_point, goal_point):
        mul_const = 1
        g_vector = goal_point - current_point
        mag_g_vector = np.linalg.norm(g_vector)
        unit_g_vector = g_vector/mag_g_vector
        desired_point = mul_const*np.array([1.25,1.25,0.25])*unit_g_vector
        
        global_desired_point = current_point + desired_point

        return global_desired_point

    def publish_next_point(self, next_point):
        next_point_msg = PoseStamped()
        next_point_msg.header.stamp = rospy.Time.now()
        next_point_msg.header.frame_id = "world"
        next_point_msg.pose.orientation.z = 0.0
        next_point_msg.pose.orientation.w = 0.0

        next_point_msg.pose.position.x = next_point[0]
        next_point_msg.pose.position.y = next_point[1]
        next_point_msg.pose.position.z = next_point[2]

        self.point_publisher.publish(next_point_msg)

    def stop_flying(self):
        motor_speed_msg = Float32()
        
        motor_speed_msg.data = 0

        self.motor_publisher0.publish(motor_speed_msg)
        self.motor_publisher1.publish(motor_speed_msg)
        self.motor_publisher2.publish(motor_speed_msg)
        self.motor_publisher3.publish(motor_speed_msg)
        self.motor_publisher4.publish(motor_speed_msg)

    