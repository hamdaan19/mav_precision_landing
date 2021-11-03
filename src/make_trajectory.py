#!/usr/bin/env python
import rospy
from klampt.model import trajectory
from klampt.io import ros as klampt_ros
from trajectory_msgs.msg import MultiDOFJointTrajectory

class MakeTrajectory():
    def __init__(self):
        self.traj_publisher = rospy.Publisher("/firefly/command/trajectory", MultiDOFJointTrajectory, queue_size=5)

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


    