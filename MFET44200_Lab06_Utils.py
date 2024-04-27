#!/usr/bin/env python3
"""
MFET 44200 - Lab 06
Contributers: Zachary Wilson

Description: A collection of utility functions for more concise code. 

'calculate_linear_dist(coord_w1 : tuple, coord_w2 : tuple) -> float'
    Takes in two arbitrary points, and calculates the distance between them.

'clamp(value : float, *, max : float, min : float) -> float'
    takes in a value, a max/min, and clamps the value between them.

    SteeringTarget:
        Description: A collection of functions used to calculate steering angles based on 
        distance form the target axis.

        'calculate_perpendicular_dist(coord_w1 : tuple, coord_w2 : tuple, coord_robot : tuple) -> float'
            takes in the x- and y-axis position from two waypoints, draws an axis between them, then calculates the robot's
            perpendicular distance to that angle.

        'calculate_waypoint_angle(coord_w1 : tuple, coord_w2 : tuple) -> float'
            take in the x- and y-axis position from two waypoints, draws an axis between them, then calculates the angle from global.

        'calculate_steering_angle(velocity : float, p_dist : float, theta_transform : float) -> float'
            takes in an arbitrary velocity, perpendicular distance, and the transform angle and produces a steering angle for the robot based on it. 

"""

import numpy as np
import math
import time

class Util:
    def calculate_linear_dist(coord_1 : tuple, coord_2 : tuple) -> float:
        #############################################################################################
        # Simple calculation to find a linear distance between two points.
        #
        # 'coord_1'    - tuple of form (x, y)
        # 'coord_2'    - tuple of form (x, y)
        
        # returns float
        # float - unsigned linear distance
        #############################################################################################
    
        x_dist = abs(coord_2[0] - coord_1[0])   # distance in x-axis
        y_dist = abs(coord_2[1] - coord_1[1])   # distance in y-axis
        
        return np.sqrt((x_dist * x_dist) + (y_dist * y_dist))   # return the hypotenuse of those distances.

    def clamp(value : float, *, max : float, min : float) -> float:
        #############################################################################################
        # Clamp function to keep a value within input bounds
        #
        # 'value'   - float; value to clamp
        # 'max'     - float; max value
        # 'min'     - float; min value
        #
        # returns float
        # float - value clamped between max and min
        #############################################################################################
    
        if value > max:         # if the value is over max
            return max          # return the max
        elif value < min:       # if the value is under min
            return min          # return the min
        return value            # otherwise it is in the bounds



    def parseWaypoints(filename : str) -> list: 
        #############################################################################################
        # Given a path file filled with waypoint data, parse that into memory for usage in code
        #
        # 'filename' - String; The filename of the path file to read.
        #
        # returns waypoint_list
        # 'waypoint_list' - list; List of all the waypoint data in order.
        #############################################################################################

        PathsOpen = open(filename, "r")
        Paths = PathsOpen.readlines()

        XPosition = []
        YPosition = []

        for i in range(len(Paths)):
            if Paths[i] == "  position:\n":
                X = Paths[i+1]
                Y = Paths[i+2]

                X = float(X[7:].strip())
                Y = float(Y[7:].strip())


                XPosition.append(X)
                YPosition.append(Y)    

        pose_arr = []
        for i in range(0, len(XPosition)):
            current_waypoint = [XPosition[i], YPosition[i], 0]
            pose_arr.append(current_waypoint)

        return pose_arr        
            

    class SteeringTarget():
        def calculate_perpendicular_dist(coord_w1 : tuple, coord_w2 : tuple, coord_robot : tuple) -> float:
            #############################################################################################
            # Calculates the robots perpendicular distance from a line drawn between the two waypoints
            # All of that math was designed in my (Zach's) desmos graph, I would not be able to explain
            # how the geometry works in just python comments.
            #
            # 'coord_w1'    - tuple of form (x, y) WORKS BEST AT (0,0)
            # 'coord_w2'    - tuple of form (x, y)
            # 'coord_robot' - tuple of form (x, y)
            # 
            # returns dist
            # 'dist'  - float; perpendicular distance, + if dist 'over', - if distance 'under'
            #############################################################################################
        
            # Make sure incoming data is valid
            assert len(coord_w1) == 2 & len(coord_w2) == 2 & len(coord_robot) == 2

            # X-INTERCEPT CALCULATION #
            # world's longest calculation
            x_intercept = ((coord_robot[0] * coord_w2[0] * coord_w2[0]) - (2 * coord_robot[0] * coord_w2[0] * coord_w1[0]) + (coord_robot[0] * coord_w1[0] * coord_w1[0]) + (coord_w2[1] * coord_w2[0] * coord_robot[1]) - (coord_w1[1] * coord_w2[0] * coord_robot[1]) - (coord_w2[1] * coord_w1[0] * coord_robot[1]) + (coord_w1[1] * coord_w1[0] * coord_robot[1]) - (coord_w2[1] * coord_w1[1] * coord_w2[0]) + (coord_w1[1] * coord_w1[1] * coord_w2[0]) + (coord_w2[1] * coord_w2[1] * coord_w1[0]) - (coord_w2[1] * coord_w1[1] * coord_w1[0])) / ((coord_w2[1] * coord_w2[1]) + (coord_w1[1] * coord_w1[1]) - (2 * coord_w2[1] * coord_w1[1]) + (coord_w2[0] * coord_w2[0]) - (2 * coord_w2[0] * coord_w1[0]) + (coord_w1[0] * coord_w1[0]))        

            # Y-INTERCEPT CALCULATION # 
            if abs(coord_w1[0] - coord_w2[0]) > 0:  # makes sure the axis isn't perfectly vertical
                y_intercept = ((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0])) * (x_intercept - coord_w1[0]) + coord_w1[1]
            elif coord_w1[0] == coord_w2[0]:        # edge case where axis is perfectly vertical
                y_intercept = coord_robot[1]
            else:                                   # if for some reason both of these fail, print error
                print("Error in calculate_perpendicular_dist: y-intercept calc recieve unexpected value")

            # PERPENDICULAR DISTANCE CALCULATION # 
            # this code feels dirty but it works
            if (coord_robot[1] >= y_intercept) & (coord_w1[0] <= coord_w2[0]):   # signs (positive) the perp. distance from the waypoint axis
                dist = np.sqrt(((coord_robot[0] - x_intercept) ** 2) + ((coord_robot[1] - y_intercept) ** 2))
            elif (coord_robot[1] >= y_intercept) & (coord_w1[0] > coord_w2[0]):   # signs (negative) the perp. distance from the waypoint axis
                dist = -np.sqrt(((coord_robot[0] - x_intercept) ** 2) + ((coord_robot[1] - y_intercept) ** 2))
            elif (coord_robot[1] < y_intercept) & (coord_w1[0] <= coord_w2[0]):  # signs (negative) the perp. distance from the waypoint axis
                dist = -np.sqrt(((coord_robot[0] - x_intercept) ** 2) + ((coord_robot[1] - y_intercept) ** 2))
            elif (coord_robot[1] < y_intercept) & (coord_w1[0] > coord_w2[0]):  # signs (positive) the perp. distance from the waypoint axis
                dist = np.sqrt(((coord_robot[0] - x_intercept) ** 2) + ((coord_robot[1] - y_intercept) ** 2))
            else:                               # if for some reason both of these fail, print error
                print("Error in calculate_perpendicular_dist: distance calc recieve unexpected value.")
            return dist



        def calculate_waypoint_angle(coord_w1 : tuple, coord_w2 : tuple) -> float:
            #############################################################################################
            # Calculates the angle of the waypoint axis with respect to global.
            # All of that math was designed in my (Zach's) desmos graph, I would not be able to explain
            # how the geometry works in just python comments.
            #
            # 'coord_w1'    - tuple of form (x, y) WORKS BEST AT (0,0)
            # 'coord_w2'    - tuple of form (x, y)
            # 
            # returns theta
            # 'theta' - float; angle from waypoint 1 -> 2; 0 -> 360
            #############################################################################################
            
            # make sure incoming data is valid
            assert len(coord_w1) >= 2 & len(coord_w2) >= 2
            
            # THETA CALCULATION #
            # All of this is just using the waypoints position and an arctan function to calculate an angle, then
            # placing that angle on the correct quadrant of the graph
            # TODO: Clean up? seems inefficent
            if coord_w2[1] >= coord_w1[1]:                              # if waypoint 2 is higher than waypoint 1
                if coord_w2[0] > coord_w1[0]:                           # if waypoint 2 is farther than waypoint 1
                    theta = np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0]))
                elif coord_w2[0] < coord_w1[0]:                         # if waypoint 2 is lesser than waypoint 1
                    theta = np.pi - np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0]))
                else:                                                   # if waypoints are equal in the X-axis
                    theta = np.pi / 2
            elif coord_w2[1] < coord_w1[1]:                             # if waypoint 2 is lower than waypoint 1
                if coord_w2[0] > coord_w1[0]:                           # if waypoint 2 is farther than waypoint 1
                    theta = (2 * np.pi) - np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0]))
                elif coord_w2[0] < coord_w1[0]:                         # if waypoint 2 is lesser than waypoint 1
                    theta = np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0])) + np.pi
                else:                                                   # if waypoints are equal in the X-axis
                    theta = (3 * np.pi) / 2
            else:                                                       # if for some reason all this fails, print an error
                print("Error in calculate_waypoint_angle: theta calculation recieved unexpected value.")
            return theta



        def calculate_steering_angle(velocity : float, p_dist : float, theta_transform : float) -> float:
            #############################################################################################
            # calculates the direction the robot should face from a calculation based on perpendicular distance
            #
            # 'velocity'        - float; governs rate angle changes, and max distance before 90 deg
            # 'p_dist'          - float; perpendicular distance from the target axis
            # 'theta_transform' - float; angle of the target axis
            #
            # returns theta
            # 'theta' - float; the angle target pointing towards the target axis; TARGET IS GLOBAL, NOT LOCAL TO THE ROBOT HEADING
            #############################################################################################
            
            # use the input velocity and the perpendicular distance to calculate the projected vector
            if abs(p_dist) <= velocity:
                v_horiz = np.sqrt((velocity ** 2) - (p_dist ** 2))      # calculation using geometry
            elif abs(p_dist) > velocity:        
                v_horiz = .000001                                       # returns basically 0, such that later calculations approach 90degs to axis
            else:
                print("Error in calculate_steering_angle: theta calculation recieved unexpected value.")

            # calculate the steering angle based off of distance and projected vector
            # this includes rotating the vector based off of the waypoint axis
            theta = np.arctan(-p_dist / v_horiz) + theta_transform 

            # Bound the theta value
            theta = math.fmod(theta, (np.pi * 2))   # mod of value with respect to 2pi (effectively bounding between [0,2pi])
            
            # legacy code, in case new code breaks it
            """
            if theta >= (np.pi * 2):            # if theta is larger than a full rotation
                theta -= (np.pi * 2)            # bound it 
            elif theta < 0:                     # if theta is a negative rotation
                theta = (np.pi * 2) + theta     # bound it
            else: 
                print("Error in calculate_steering_angle: Theta bounding recieved unexpected value.")
            """
            return theta

        def global_to_local_steering(robot_angle : float, steering_target : float) -> float:
            theta_calc = robot_angle - steering_target
            if theta_calc > np.pi:
                theta_calc -= (2 * np.pi)
            elif theta_calc < -np.pi:
                theta_calc += (2 * np.pi)

            return theta_calc
    


    class Interpolation:
        def pose_interp(current_pose_estimate : list, current_angle : float, current_vel : float, last_update : float) -> list:
            current_time = time.monotonic()
            travel_dist = current_vel * (current_time - last_update)
            step_vect = Util.Interpolation.step_vector(travel_dist, current_angle)
            
            new_estimate = [(current_pose_estimate[0] + step_vect[0]), (current_pose_estimate[1] + step_vect[1]), current_pose_estimate[2]]
            return [new_estimate, current_time]

        def step_vector(dist : float, angle : float) -> list:
            if angle == (np.pi / 2):
                return [0, dist]
            if angle == ((3 * np.pi) / 2):
                return [0, -dist]
            
            x = dist * np.cos(angle)
            y = dist * np.sin(angle)
            return [x, y]