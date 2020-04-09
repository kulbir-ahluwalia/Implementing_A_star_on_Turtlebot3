import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
from time import process_time


from intersection_check import *


def cart2img(adjust_coord):
    return [adjust_coord[0], 200 - adjust_coord[1]]

# function returns false when the point is outside the circle
def circular_obstacle(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (225, 150)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance

    distance_from_center = ((test_point_coord_x - circle_center[0]) ** 2 + (
            test_point_coord_y - circle_center[1]) ** 2) ** 0.5

    if distance_from_center > (25 + augment_distance):
        return False
    else:
        return True


# function returns false when the point is outside the ellipse
def ellipsoid_obstacle(clearance, radius_rigid_robot, test_point_coord):
    ellipsoid_center = (150, 100)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance
    semi_major_axis = 40
    semi_minor_axis = 20

    distance_from_center = ((test_point_coord_x - ellipsoid_center[0]) ** 2) / (
            (semi_major_axis + augment_distance) ** 2) + (test_point_coord_y - ellipsoid_center[1]) ** 2 / (
                                   (semi_minor_axis + augment_distance) ** 2)

    if distance_from_center > 1:
        return False
    else:
        return True


def rectangle_obstacle(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (225, 150)
    augment_distance = radius_rigid_robot + clearance

    rectangle_point_1 = [100, 38.66025]
    rectangle_point_2 = [35.0481, 76.1603]
    rectangle_point_3 = [30.0481, 67.5]
    rectangle_point_4 = [95, 30]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the rectangle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_1, rectangle_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (
            edge1_m_c[1] + (augment_distance * 2 / (3 ** 0.5)))
    # print(line1)
    if line1 >= 0:
        flag1 = False
        # print("False")
    else:
        flag1 = True
        # print("True")

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_2, rectangle_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance * 2))
    # print(line2)
    if line2 >= 0:
        flag2 = False
        # print("False")
    else:
        flag2 = True
        # print("True")

    edge3_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_3, rectangle_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (
            edge3_m_c[1] - (augment_distance * 2 / (3 ** 0.5)))
    # print(line3)
    if line3 >= 0:
        flag3 = True
        # print("True")
    else:
        flag3 = False
        # print("False")

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_4, rectangle_point_1)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance * 2))
    # print(line4)
    if line4 >= 0:
        flag4 = True
        # print("True")
    else:
        flag4 = False
        # print("False")

    if flag1 and flag2 and flag3 and flag4:
        return True
    else:
        return False


def rhombus_obstacle(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance

    rhombus_point_1 = [250, 25]
    rhombus_point_2 = [225, 40]
    rhombus_point_3 = [200, 25]
    rhombus_point_4 = [225, 10]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the rectangle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_1, rhombus_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (edge1_m_c[1] + (augment_distance / 0.8575))
    # print(line1)
    if line1 >= 0:
        flag1 = False
    else:
        flag1 = True

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_2, rhombus_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance / 0.8575))
    # print(line2)
    if line2 >= 0:
        flag2 = False
    else:
        flag2 = True

    edge3_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_3, rhombus_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (edge3_m_c[1] - (augment_distance / 0.8575))
    # print(line3)
    if line3 >= 0:
        flag3 = True
    else:
        flag3 = False

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_4, rhombus_point_1)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance / 0.8575))
    # print(line4)
    if line4 >= 0:
        flag4 = True
    else:
        flag4 = False

    if flag1 and flag2 and flag3 and flag4:
        return True
    else:
        return False


def nonconvex_obstacle_right_half(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance

    nonconvex_point_1 = [100, 150]
    nonconvex_point_2 = [75, 185]
    nonconvex_point_3 = [60, 185]
    nonconvex_point_4 = [50, 150]
    nonconvex_point_5 = [75, 120]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the nonconvex_obstacle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_1, nonconvex_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (edge1_m_c[1] + (augment_distance / 0.58124))
    # print(line1)
    if line1 >= 0:
        flag1 = False
        # print("False")
    else:
        flag1 = True
        # print("True")

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_2, nonconvex_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance / 1))
    # print(line2)
    if line2 >= 0:
        flag2 = False
        # print("False")
    else:
        flag2 = True
        # print("True")

    # edge 3 is not augmented with clearance+robot_radius since its inside the nonconvex polygon
    edge3_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_3, nonconvex_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (edge3_m_c[1] + (augment_distance / 0.27472))
    # print(line3)
    if line3 >= 0:
        flag3 = False
        # print("False")
    else:
        flag3 = True
        # print("True")

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_4, nonconvex_point_5)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance / 0.64018))
    # print(line4)
    if line4 >= 0:
        flag4 = True
        # print("True")
    else:
        flag4 = False
        # print("False")

    edge5_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_5, nonconvex_point_1)
    line5 = test_point_coord[1] - (edge5_m_c[0] * test_point_coord[0]) - (edge5_m_c[1] - (augment_distance / 0.640184))
    # print(line4)
    if line5 >= 0:
        flag5 = True
        # print("True")
    else:
        flag5 = False
        # print("False")

    if flag1 and flag2 and flag3 and flag4 and flag5:
        return True
    else:
        return False


def nonconvex_obstacle_left_half(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance

    nonconvex_point_1 = [50, 150]
    nonconvex_point_2 = [60, 185]
    nonconvex_point_3 = [25, 185]
    nonconvex_point_4 = [20, 120]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the nonconvex_obstacle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_1, nonconvex_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (edge1_m_c[1] - (augment_distance / 0.27472))
    # print(line1)
    if line1 >= 0:
        flag1 = True
        # print("True")
    else:
        flag1 = False
        # print("False")

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_2, nonconvex_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance / 1))
    # print(line2)
    if line2 >= 0:
        flag2 = False
        # print("False")
    else:
        flag2 = True
        # print("True")

    # edge 3 is not augmented with clearance+robot_radius since its inside the nonconvex polygon
    edge3_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_3, nonconvex_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (edge3_m_c[1] + (augment_distance / 0.0767))
    # print(line3)
    if line3 >= 0:
        flag3 = False
        # print("False")
    else:
        flag3 = True
        # print("True")

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_4, nonconvex_point_1)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance / 0.7071))
    # print(line4)
    if line4 >= 0:
        flag4 = True
        # print("True")
    else:
        flag4 = False
        # print("False")

    if flag1 and flag2 and flag3 and flag4:
        return True
    else:
        return False


def boundary_obstacle(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance
    x = test_point_coord[0]
    y = test_point_coord[1]

    if -5.1 <= x < augment_distance:
        return True
    elif (5.1 - augment_distance) < x <= 5.1:
        return True
    elif -5.1 <= y < augment_distance:
        return True
    elif (5.1 - augment_distance) < y <= 5.1:
        return True
    else:
        return False




def test_point_obstacle_check(clearance, radius_rigid_robot, test_point_coord, image):
    #test_point_coord = cart2img(test_point_coord)
    if circular_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif ellipsoid_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif rectangle_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif rhombus_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif nonconvex_obstacle_right_half(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif nonconvex_obstacle_left_half(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif boundary_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    else:
        return False

