#!/usr/bin/env python3
"""
MFET 44200 - Lab 06
Contributers: Zachary Wilson

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

def calculate_linear_dist(coord_1 : tuple, coord_2 : tuple) -> float:
    x_dist = abs(coord_2[0] - coord_1[0])
    y_dist = abs(coord_2[1] - coord_1[1])
    
    return np.sqrt(x_dist ** 2 + y_dist ** 2)



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
   
    # X-intercept calculation
    # world's longest calculation
    x_intercept = ((coord_robot[0] * coord_w2[0] * coord_w2[0]) - (2 * coord_robot[0] * coord_w1[0] * coord_w1[0]) + (coord_robot[0] * coord_w1[0] * coord_w1[0]) + (coord_w2[1] * coord_w2[0] * coord_robot[1]) - (coord_w1[1] * coord_w2[0] * coord_robot[1]) - (coord_w2[1] * coord_w1[0] * coord_robot[1]) + (coord_w1[1] * coord_w1[0] * coord_robot[1]) - (coord_w2[1] * coord_w1[1] * coord_w2[0]) + (coord_w1[1] * coord_w1[1] * coord_w2[0]) + (coord_w2[1] * coord_w2[1] * coord_w1[0]) - (coord_w2[1] * coord_w1[1] * coord_w1[0])) / ((coord_w2[1] * coord_w2[1]) + (coord_w1[1] * coord_w1[1]) - (2 * coord_w2[1] * coord_w1[1]) + (coord_w2[0] * coord_w2[0]) - (2 * coord_w2[0] * coord_w1[0]) + (coord_w1[0] * coord_w1[0]))        

    # y-intercept calculation
    if abs(coord_w1[0] - coord_w2[0]) > 0:  # makes sure the axis isn't perfectly vertical
        y_intercept = ((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0])) * (x_intercept - coord_w1[0]) + coord_w1[1]
    elif coord_w1[0] == coord_w2[0]:        # edge case where axis is perfectly vertical
        y_intercept = coord_robot[1]
    else:                                   # if for some reason both of these fail, print error
        print("Error in calculate_perpendicular_dist: y-intercept calc recieve unexpected value")

    # perpendicular distance calculation
    # this code feels dirty but it works
    if (coord_robot[1] >= y_intercept) & (coord_w1[0] <= coord_w2[0]):   # signs (positive) the perp. distance from the waypoint axis
        dist = np.sqrt(((coord_robot[0] - x_intercept) ** 2) + ((coord_robot[1] - y_intercept) ** 2))
    elif (coord_robot[1] >= y_intercept) & (coord_w1[0] > coord_w2[0]):   # signs (negative) the perp. distance from the waypoint axis
        dist = -np.sqrt(((coord_robot[0] - x_intercept) ** 2) + ((coord_robot[1] - y_intercept) ** 2))
    elif coord_robot[1] < y_intercept & (coord_w1[0] <= coord_w2[0]):  # signs (negative) the perp. distance from the waypoint axis
        dist = -np.sqrt(((coord_robot[0] - x_intercept) ** 2) + ((coord_robot[1] - y_intercept) ** 2))
    elif coord_robot[1] < y_intercept & (coord_w1[0] > coord_w2[0]):  # signs (positive) the perp. distance from the waypoint axis
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
    # theta calculation
    
    # All of this is just using the waypoints position and an arctan function to calculate an angle, then
    # placing that angle on the correct quadrant of the graph

    # TODO: Clean up? seems inefficent
    if coord_w2[1] >= coord_w1[1]:                              # if waypoint 2 is higher than waypoint 1
        if coord_w2[0] > coord_w1[0]:                           # if waypoint 2 is farther than waypoint 1
            theta = np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0]))
        elif coord_w2[0] < coord_w1[0]:                         # if waypoint 2 is lesser than waypoint 1
            theta = np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0])) + np.pi
        else:                                                   # if waypoints are equal in the X-axis
            theta = np.pi / 2
    elif coord_w2[1] < coord_w1[1]:                             # if waypoints are equal in the Y-axis
        if coord_w2[0] > coord_w1[0]:                           # if waypoint 2 is farther than waypoint 1
            theta = np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0])) + np.pi
        elif coord_w2[0] < coord_w1[0]:                         # if waypoint 2 is lesser than waypoint 1
            np.arctan((coord_w2[1] - coord_w1[1]) / (coord_w2[0] - coord_w1[0])) + (2 * np.pi)
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
        v_horiz = np.sqrt((velocity ** 2) - (p_dist ** 2))
    elif abs(p_dist) > velocity:
        v_horiz = .000001
    else:
        print("Error in calculate_steering_angle: theta calculation recieved unexpected value.")

    # calculate the steering angle based off of distance and projected vector
    # this includes rotating the vector based off of the waypoint axis
    theta = np.arctan(-p_dist / v_horiz) + theta_transform 

    # Bound the theta value
    if theta >= (np.pi * 2):            # if theta is larger than a full rotation
        theta -= (np.pi * 2)            # bound it 
    elif theta < 0:                     # if theta is a negative rotation
        theta = (np.pi * 2) + theta     # bound it
    else: 
        print("Error in calculate_steering_angle: Theta bounding recieved unexpected value.")
    return theta