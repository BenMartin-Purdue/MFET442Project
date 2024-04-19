#!/usr/bin/env python3

"""
MFET 44200 - Lab 06
Contributers: Zachary Wilson

Description: The main file of Lab 06 that controls the acutal movement and navigation of the 
    rally car based on incoming data from LIDAR, odometry, and AMCL position localization. 
    The MapFollower class handles the intake of position data from the AMCL localizer, and then
    makes use of geometric equations to figure out a target angle vector and magnitude of the 
    velocity to keep a stable tracking of inbuilt map waypoints. Finally, it uses these targets
    in a PID controller that controls the actual steering angle and throttle.

    4/16/2024-2:51:35 - Final draft of the code, before actual testing on the rallycar.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from simple_pid import PID
from MFET44200_Lab06_Utils import Util

class MapFollower:
    def __init__(self):
        #############################################################################################
        # General initalization of the MapFollower class
        #############################################################################################    
        self.waypoints = Util.parseWaypoints()
        self.index = 1
        self.desired_waypoint = self.waypoints.pose_arr[self.index]
        self.previous_waypoint = self.waypoints.pose_arr[self.index - 1]

        # Node declaration
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_current_pose)
        # TODO: This needs to be implemented to actually use PID control instead of simulating it
        # rospy.Subscriber("/odom", Odometry, self.update_current_vel)
        self.accel_pub = rospy.Publisher("/accelerator_cmd", Float32, queue_size=10)
        self.steer_pub = rospy.Publisher("/steering_cmd", Float32, queue_size=10)
        rospy.Rate(30) # 30Hz

        # Local control information
        self.current_pose = [0,0,0]
        self.current_vel = 0
        self.current_steering_angle = 0

        # PID controller
        # CONTROL PARAMETERS #
        accel_Kp = 1
        accel_Ki = .0001
        accel_Kd = .0009

        steer_Kp = 1
        steer_Ki = .0001
        steer_Kd = .0009

        self.max_vel = 512
        self.min_vel = 256
        self.velocity_falloff = 5
        self.steering_vel = 20
        # CONTROL PARAMETERS # 

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
        rospy.loginfo(self.current_pose)



    def update_current_vel(self, msg) -> None:
        # HACK: No idea if this velocity is calculated in global space or local space.
        #       Gonna be WAY easier if its local -> I don't wanna do more geometry

        self.current_vel = msg.TwistWithCovariance.Twist.linear.x



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
        
        p_dist = Util.SteeringTarget.calculate_perpendicular_dist(waypoint_1_tup, waypoint_2_tup, robot_pos_tup)    # calculates the perpendicular distance
        theta_transform = Util.SteeringTarget.calculate_waypoint_angle(waypoint_1_tup, waypoint_2_tup)              # calculates the angle between the two waypoints in global space
        steering_angle_global = Util.SteeringTarget.calculate_steering_angle(vel, p_dist, theta_transform)          # calculates a smooth steering angle based on arbitary velocity and previous calculation
        steering_angle_local = math.radians(self.current_pose[2]) - steering_angle_global       # converts the global steering angle back to local space

        return steering_angle_local



    def process_velocity_target(self, waypoint_target : list, robot_pos : list, vel_max : float, vel_min_percent: float, vel_droppoff) -> float:
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
       
        waypoint_2_tup = (waypoint_target[0], waypoint_target[1])                                   # converts data to be readable by following methods
        robot_pos_tup = (self.current_pose[0], self.current_pose[1])
        
        linear_comp =  np.exp(-(vel_droppoff / Util.calculate_linear_dist(robot_pos_tup, waypoint_2_tup)))  # Drop off velocity percentage as car approaches the waypoint
        angular_comp = (np.pi / 2) - abs(self.current_steering_angle)                                    # Drop off velocity as percentage as car deviates from the path
        vel_final = vel_max * ((linear_comp * angular_comp) / (np.pi / 2))                          # combines these calculations and max 

        return vel_final


    def control_loop(self, waypoint_tolerance : float = 20, will_loop : bool = True) -> None:
        #############################################################################################
        # The "Main" loop of the rallycar control. Makes use incoming data to update the current waypoint,
        # calculate the heading for the rallycar steering, and then use a PID controller for both
        # the acceleration and the steering angle. 
        #
        # 'waypoint_tolerance' - float; keyword argument that controls how close to a waypoint the car can get before its considered 'reached'
        # 'will_loop'          - bool;  keyword argument that controls if the car will target the original waypoint once it reaches the last. 
        #
        # TODO: Figure out how to run this at 10 / 30 / 50 / variable Hz
        #           *Probably not included inside the function.
        # TODO: Messy Comments, need to clean up
        # 
        #############################################################################################    

        self.vel_error = self.accel_PID(self.current_vel - self.process_velocity_target(self.desired_waypoint, self.current_pose, self.max_vel, self.min_vel/self.max_vel, self.velocity_falloff))
        self.steer_error = self.steering_PID(self.current_steering_angle - self.process_steering_target(self.previous_waypoint, self.desired_waypoint, self.current_pose, self.steering_vel))
        
        # Calculating tolerance 
        dist_to_desired_waypoint = Util.calculate_linear_dist((self.current_pose[0], self.current_pose[1]), (self.desired_waypoint[0], self.desired_waypoint[1]))
        
        # GATEWAY CONDITION #
        if(dist_to_desired_waypoint > waypoint_tolerance):
            # should the distance to the desired waypoint be greater than the tolerance, exit now.
            return
        
        # LOOPBACK CONDITIONS #
        if((self.index + 1 >= len(self.waypoints)) and will_loop == True): 
            # if index is larger than the waypoint array, reset the index back to 0
            # basically, will call a continous looping of the waypoints if true
            self.index = -1 # <- will have one added to it - making it 0
        elif (self.index + 1 >= len(self.waypoints) and will_loop == False):
            # HACK: This avoids an indexOutOfBoundsException, but never actually closes down the 
            #   control loop.
            # TODO: Actually close the control loop.
            # stops the loop from updating if the end of the array has been reached            
            return

        # Stepping the desired waypoint 
        self.index += 1                                             # step the index
        self.previous_waypoint = self.desired_waypoint              # save the previous desired waypoint
        self.desired_waypoint = self.waypoints[self.index] # set the new desired waypoint
        


    def write_accel(self) -> None:
        #############################################################################################
        # Writes the updated velocity to the accelerator. Also saves this updated velocity as the most 
        # recent velocity
        #############################################################################################    
        target_vel = self.current_vel + self.vel_error
        target_vel = Util.clamp(target_vel, max=2048, min=-2048)
        self.current_vel = target_vel
        target_vel = Float32(target_vel)
        self.accel_pub.publish(target_accel)



    
    def write_steering(self) -> None: 
        #############################################################################################
        # Writes the updated steering angle to the steering. Also saves this updated angle as the most 
        # recent angle
        #############################################################################################    
        target_steering = self.current_steering_angle + self.steer_error
        target_steering = Util.clamp(target_steering, max=2048, min=-2048)
        self.current_steering_angle = target_steering
        target_steering = Float32(target_steering)
        self.steer_pub.publish(target_steering)


def main():
    map_follower_instance = MapFollower()
    while not rospy.is_shutdown():
        map_follower_instance.control_loop()
        map_follower_instance.write_accel()
        map_follower_instance.write_steering()
        rospy.spin()

if __name__ == "__main__":
    main()
