#!/usr/bin/env python3
"""
MFET 44200 - Lab 06
Contributers: Zachary Wilson

Description: A collection of functions used to calculate steering angles based on 
distance form the target axis.

'calculate_linear_dist(coord_w1 : tuple, coord_w2 : tuple) -> float'
    Takes in two arbitrary points, and calculates the distance between them.

'clamp(value : float, *, max : float, min : float) -> float'
    takes in a value, a max/min, and clamps the value between them.

"""

import numpy as np

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
    
    return np.sqrt(x_dist ** 2 + y_dist ** 2)   # return the hypotenuse of those distances.

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
