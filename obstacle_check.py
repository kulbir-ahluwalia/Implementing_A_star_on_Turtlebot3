import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
from time import process_time


#from intersection_check import *

def find_line_slope_and_intercept(test_point_coord, line_point_1, line_point_2):
    if (line_point_2[0] - line_point_1[0]) == 0:
        x_intercept = line_point_1[0]
        return math.inf, x_intercept
    else:
        slope = (line_point_2[1] - line_point_1[1]) / (line_point_2[0] - line_point_1[0])
        y_intercept = line_point_1[1] - (slope * line_point_1[0])
        return slope, y_intercept


def cart2img(adjust_coord):
    return [adjust_coord[0], 5.1 - adjust_coord[1]]

# function returns false when the point is outside the circle
def circular_obstacle_1(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (2, 3)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance

    distance_from_center = ((test_point_coord_x - circle_center[0]) ** 2 + (
            test_point_coord_y - circle_center[1]) ** 2) ** 0.5

    if distance_from_center > (1 + augment_distance):
        return False
    else:
        return True

def circular_obstacle_2(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (0, 0)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance

    distance_from_center = ((test_point_coord_x - circle_center[0]) ** 2 + (
            test_point_coord_y - circle_center[1]) ** 2) ** 0.5

    if distance_from_center > (1 + augment_distance):
        return False
    else:
        return True

def circular_obstacle_3(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (-2,-3)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance

    distance_from_center = ((test_point_coord_x - circle_center[0]) ** 2 + (
            test_point_coord_y - circle_center[1]) ** 2) ** 0.5

    if distance_from_center > (1 + augment_distance):
        return False
    else:
        return True

def circular_obstacle_4(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (2,-3)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance

    distance_from_center = ((test_point_coord_x - circle_center[0]) ** 2 + (
            test_point_coord_y - circle_center[1]) ** 2) ** 0.5

    if distance_from_center > (1 + augment_distance):
        return False
    else:
        return True

def square_obstacle_1(clearance, radius_rigid_robot, test_point_coord):

    augment_distance = radius_rigid_robot + clearance
    square_point_1 = [-1.25,3.75]
    square_point_2 = [-2.75,3.75]
    square_point_3 = [-2.75,2.25]
    square_point_4 = [-1.25,2.25]

    if test_point_coord[0]<=square_point_1[0]+augment_distance and test_point_coord[0]>=square_point_2[0]-augment_distance:
        flag1 = True
        # print("True")
    else:
        flag1 = False
        # print("False")

    if test_point_coord[1]<=square_point_1[1]+augment_distance and test_point_coord[1]>=square_point_3[1]-augment_distance:
        flag2 = True
        # print("True")
    else:
        flag2 = False
        # print("False")

    if flag1 and flag2:
        return True
    else:
        return False

def square_obstacle_2(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance
    square_point_1 = [-3.25,0.75]
    square_point_2 = [-4.75,0.75]
    square_point_3 = [-4.75,-0.75]
    square_point_4 = [-3.25,-0.75]

    if test_point_coord[0]<=square_point_1[0]+augment_distance and test_point_coord[0]>=square_point_2[0]-augment_distance:
        flag1 = True
        # print("True")
    else:
        flag1 = False
        # print("False")

    if test_point_coord[1]<=square_point_1[1]+augment_distance and test_point_coord[1]>=square_point_3[1]-augment_distance:
        flag2 = True
        # print("True")
    else:
        flag2 = False
        # print("False")

    if flag1 and flag2:
        return True
    else:
        return False


def square_obstacle_3(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance
    square_point_1 = [4.75,0.75]
    square_point_2 = [3.25,0.75]
    square_point_3 = [3.25,-0.75]
    square_point_4 = [4.75,-0.75]

    if test_point_coord[0]<=square_point_1[0]+augment_distance and test_point_coord[0]>=square_point_2[0]-augment_distance:
        flag1 = True
        # print("True")
    else:
        flag1 = False
        # print("False")

    if test_point_coord[1]<=square_point_1[1]+augment_distance and test_point_coord[1]>=square_point_3[1]-augment_distance:
        flag2 = True
        # print("True")
    else:
        flag2 = False
        # print("False")

    if flag1 and flag2:
        return True
    else:
        return False


def boundary_obstacle(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance
    x = test_point_coord[0]
    y = test_point_coord[1]

    if -5.1 <= x <= -5.1+augment_distance:
        return True
    elif (5.1 - augment_distance) <= x <= 5.1:
        return True
    elif -5.1 <= y <= -5.1+augment_distance:
        return True
    elif (5.1 - augment_distance) <= y <= 5.1:
        return True
    else:
        return False

#
#
#
def test_point_obstacle_check(clearance, radius_rigid_robot, test_point_coord):
    #test_point_coord = cart2img(test_point_coord)
    if circular_obstacle_1(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif circular_obstacle_2(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif circular_obstacle_3(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif circular_obstacle_4(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif square_obstacle_1(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif square_obstacle_2(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif square_obstacle_3(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif boundary_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    else:
        return False
#


# print(square_obstacle_1(0.1,0.1,(-2,3)))   #inside square 1, hence output is true
# print(square_obstacle_1(0.1,0.1,(-2.75,3.75)))   #on boundary of square 1, hence output is true
# print(square_obstacle_1(0,0,(0,0))) #outside, hence false


# print(square_obstacle_2(0.1,0.1,(-4,0)))   #inside square 1, hence output is true
# print(square_obstacle_2(0.1,0.1,(-4.75,-0.75)))   #on boundary of square 1, hence output is true
# print(square_obstacle_2(0,0,(0,0)))   #outside, hence false


# print(square_obstacle_3(0.1,0.1,(4,0)))   #inside square 1, hence output is true
# print(square_obstacle_3(0.1,0.1,(4.75,0.75)))   #on boundary of square 1, hence output is true
# print(square_obstacle_3(0,0,(0,0)))   #outside, hence false

# print(test_point_obstacle_check(0,0,(0,0),None))
# print(test_point_obstacle_check(0,0,(5,5),None))