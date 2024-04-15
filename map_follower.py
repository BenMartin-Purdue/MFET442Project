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
import numpy as np
from simple_pid import PID
from MFET44200_Lab06_Utils import calculate_linear_dist
from MFET44200_Lab06_Steering import calculate_perpendicular_dist, calculate_steering_angle, calculate_waypoint_angle

class MapFollower:
    def __init__(self):
        #############################################################################################
        # General initalization of the MapFollower class
        #############################################################################################    
        self.waypoints = waypoint()
        self.index = 1
        self.desired_waypoint = self.waypoints.pose_arr[self.index]
        self.previous_waypoint = self.waypoints.pose_arr[self.index - 1]

        rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.update_current_pose)
        self.current_pose = []

        # PID controller
        
        # control parameters
        accel_Kp = 0
        accel_Ki = 0
        accel_Kd = 0

        steer_Kp = 0
        steer_Ki = 0
        steer_Kd = 0

        self.set_vel = 600
        self.steering_vel = 20
        # Control parameters

        self.accel_PID = PID(accel_Kp, accel_Ki, accel_Kd, 0)               # import the parameters into the PID controller
        self.steering_PID = PID(steer_Kp, steer_Ki, steer_Kd, 0)            # Set point is 0 here, because later calculations evaluate to 0 when on base.



    def update_current_pose(self, msg) -> None:
        #############################################################################################
        # Updates the current position by taking data from the amcl localization, and extracts usable
        # data from the output data object.
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



    def process_steering_target(self, waypoint_1 : list, waypoint_2 : list, robot_pos : list, vel: float) -> float:
        #############################################################################################
        # Takes in the waypoint data, the robot's current position, and an arbitary velocity value, 
        # and returns a floating point angle for the car to target in radians.
        # 
        # 'waypoint_1'      - list format of data extracted from the waypoint file; first target
        # 'waypoint_2'      - list format of data extracted from the waypoint file; second target
        # 'robot_pos'       - list format of data gotten from AMCL; robot's current position
        # 'vel'             - float; arbitary value that determines how quickly the robot levels out to being 'on axis'
        #
        # returns steering_angle_local
        # 'steering_angle_local'    - float, Radians; the calculated steering angle for the rally car to target.
        #############################################################################################    
        
        waypoint_1_tup = (waypoint_1[0], waypoint_1[1])
        waypoint_2_tup = (waypoint_2[0], waypoint_2[1])                 # converts data to be readable by following methods
        robot_pos_tup = (self.current_pose[0], self.current_pose[1])
        
        p_dist = calculate_perpendicular_dist(waypoint_1_tup, waypoint_2_tup, robot_pos_tup)    # calculates the perpendicular distance
        theta_transform = calculate_waypoint_angle(waypoint_1_tup, waypoint_2_tup)              # calculates the angle between the two waypoints in global space
        steering_angle_global = calculate_steering_angle(vel, p_dist, theta_transform)          # calculates a smooth steering angle based on arbitary velocity and previous calculation
        steering_angle_local = math.radians(self.current_pose[2]) - steering_angle_global       # converts the global steering angle back to local space

        return steering_angle_local



    def process_velocity_target(self, waypoint_target : list, robot_pos : list, cur_steering_angle : float, vel_max : float, vel_min_percent: float, vel_droppoff) -> float:
        #############################################################################################
        # Takes in the waypoint data, the robot's current position, and velocity parameters, 
        # and returns a floating point velocity for the car to target.
        # 
        # 'waypoint_target'     - list format of data extracted from the waypoint file; target waypoint
        # 'robot_pos'           - list format of data gotten from AMCL; robot's current position
        # 'vel_max'             - float; arbitrary value that determines the car's max speed
        # 'vel_min_percent'     - float; arbitrary value that determines the slowest speed the car will decell to as a percentage of the max
        # 'vel_droppoff'        - float; arbitrary value that determines how soon the car will decelerate as it appreaches the waypoint
        #
        # returns vel_final
        # 'vel_final'           - float; the calculated velocity for the rally car to target.
        ############################################################################################# 
       
        waypoint_2_tup = (waypoint_target[0], waypoint_target[1])                 # converts data to be readable by following methods
        robot_pos_tup = (self.current_pose[0], self.current_pose[1])
        
        linear_comp =  np.exp(vel_droppoff / calculate_linear_dist(robot_pos_tup, waypoint_2_tup))  # Drop off velocity percentage as car approaches the waypoint
        angular_comp = (np.pi / 2) - cur_steering_angle                                             # Drop off velocity as percentage as car deviates from the path
        vel_final = vel_max * ((linear_comp * angular_comp) / (np.pi / 2))                          # combines these calculations and max 

        return vel_final



    def control_loop(self) -> None:
        #############################################################################################
        # The "Main" loop of the rallycar control. Makes use incoming data to update the current waypoint,
        # calculate the heading for the rallycar steering, and then use a PID controller for both
        # the acceleration and the steering angle. 
        # 
        # TODO: Figure out how to run this at 10 / 30 / 50 / variable Hz
        #           *Probably not included inside the function.
        # TODO: Messy Comments, need to clean up
        # 
        #############################################################################################    

        # TODO: THIS NEEDS TO BE POPULATED BEFORE THIS WORKS!!!!
        current_vel = None
        current_steering_angle = None
        # HACK: current_vel and current_steering_angle are not provided yet! This code stops the interperter from yelling but it does not work yet!
        vel_error = self.accel_PID(current_vel - self.process_velocity_target(self.desired_waypoint, self.current_pose, current_steering_angle, .2, 5))
        steer_error = self.steering_PID(current_steering_angle - self.process_steering_target(self.previous_waypoint, self.desired_waypoint, self.current_pose, self.steering_vel))

        # Waypoint params
        waypoint_tolerance = 20 # defines the radial distance required to "complete" a waypoint
        will_loop = True        # defines if the robot will target waypoint 0 once it reaches the final waypoint, or terminate
        # Waypoint params

        # Calculating tolerance 
        dist_to_desired_waypoint = calculate_linear_dist((self.current_pose[0], self.current_pose[1]), (self.desired_waypoint[0], self.desired_waypoint[1]))
        
        # Gateway condition
        if(dist_to_desired_waypoint > waypoint_tolerance):
            # should the distance to the desired waypoint be greater than the tolerance, exit now.
            return
        
        # loopback conditions
        if((self.index > len(self.waypoints.pose_arr)) and will_loop == True): 
            # if index is larger than the waypoint array, reset the index back to 0
            # basically, will call a continous looping of the waypoints if true
            self.index = -1 # will have one added to it - making it 0
        elif (self.index > len(self.waypoints.pose_arr) and will_loop == False):
            # HACK: This avoids an indexOutOfBoundsException, but never actually closes down the 
            #   control loop.
            # TODO: Actually close the control loop.
            # stops the loop from updating if the end of the array has been reached            
            return

        # Stepping the desired waypoint 
        self.index += 1                                             # step the index
        self.previous_waypoint = self.desired_waypoint              # save the previous desired waypoint
        self.desired_waypoint = self.waypoints.pose_arr[self.index] # set the new desired waypoint


