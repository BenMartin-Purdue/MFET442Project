#!/usr/bin/env python3
"""
MFET 44200 - Lab 06
Contributers: Zachary Wilson

Description: The main file of Lab 06 that controls the acutal movement and navigation of the 
    rally car based on incoming data from LIDAR, odometry, and AMCL position localization.  

TODO: Add more

"""

import waypoint
import rospy
import geometry_msgs.msg
import tf
import math
from MFET44200_Lab06_Steering import calculate_linear_dist, calculate_perpendicular_dist, calculate_steering_angle, calculate_waypoint_angle

class MapFollower:
    def __init__(self):
    #############################################################################################
    # 
    #############################################################################################    
        self.waypoints = waypoint()
        self.index = 1
        self.desired_waypoint = self.waypoints.pose_arr[self.index]
        self.previous_waypoint = self.waypoints.pose_arr[self.index - 1]

        rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.update_current_pose)
        self.current_pose = []



    def update_current_pose(self, msg):
    #############################################################################################
    # 
    #############################################################################################    
        quaternion = (
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,    # extracting data from the pose
			msg.pose.pose.orientation.z,    # to a tuple for processing
			msg.pose.pose.orientation.w
        )
        
        euler = tf.transformations.euler_from_quaternion(quaternion)    # I dont know what this does
        yaw = math.degrees(euler[2])                                    # but apperently it gives a yaw value
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]    # Set the current position


    def control_loop(self, msg) -> None:
    #############################################################################################
    #
    # 
    # TODO: Figure out how to run this at 10Hz
    # 
    #############################################################################################    
        
        # changable params
        waypoint_tolerance = 20
        will_loop = True

        # calculating tolerance 
        dist_to_desired_waypoint = calculate_linear_dist((self.current_pose[0], self.current_pose[1]), (self.desired_waypoint[0], self.desired_waypoint[1]))
        
        # gateway condition
        if(dist_to_desired_waypoint > waypoint_tolerance):
            # should the distance to the desired waypoint be greater than the tolerance, exit now.
            return
        
        # loopback conditions
        if((self.index > len(self.waypoints.pose_arr)) and will_loop == True): 
            # if index is larger than the waypoint array, reset the index back to 0
            # basically, will call a continous looping of the waypoints if true
            self.index = 0

        elif (self.index > len(self.waypoints.pose_arr) and will_loop == False):
            # HACK: This avoids an indexOutOfBoundsException, but never actually closes down the 
            #   control loop.
            # TODO: Actually close the control loop.
            # stops the loop from updating if the end of the array has been reached
            return

        # Stepping the desired waypoint 
        self.index += 1
        self.previous_waypoint = self.desired_waypoint
        self.desired_waypoint = self.waypoints.pose_arr[self.index]
    
