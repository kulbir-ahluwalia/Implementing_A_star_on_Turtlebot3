import math
import matplotlib.pyplot as plt
import numpy as np
from obstacle_check import *


fig, ax = plt.subplots()

rpm1 = 40
rpm2 = 40
test_point_coord = (-3.5,-3)
test_point_angle = 30


action_set = [[rpm2, rpm1],
              [0,rpm1],
              [0,rpm2],
              [rpm1,rpm1],
              [rpm2,rpm2],
              [rpm1,0],
              [rpm2,0],
              [rpm1, rpm2]]


# def plot_curve(Xi, Yi, Thetai, UL, UR):
r = 0.033
L = 0.160
pi = math.pi
# print(pi)

clearance = 0.2
radius_rigid_robot = 0.105
augment_distance = radius_rigid_robot + clearance

def rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm1, rpm2):
    t = 0
    dt = 0.1
    Xn = test_point_coord[0]
    Yn = test_point_coord[1]
    Thetan = pi * test_point_angle/ 180

    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn = Xn + (r * (2 /60) * pi *(rpm1 + rpm2) * math.cos(Thetan) * dt)/2
        Yn = Yn + (r * (2 /60) * pi *(rpm1 + rpm2) * math.sin(Thetan) * dt)/2
        Thetan = Thetan + ((r * (2 /60) * pi / L) * (rpm2 - rpm1) * dt)
        # plt.plot([Xs, Xn], [Ys, Yn], color="blue")

    Thetan = 180 * (Thetan) / pi
    new_point = [Xn, Yn, Thetan]

    action_cost = 1

    if (test_point_obstacle_check(clearance, radius_rigid_robot, new_point)) == False:
        return new_point, action_cost
    else:
        return None, None



# print(rpm_1_1(0.2,       0.105,              (-3.5,-3),         0,               rpm1,   rpm2))
#             (clearance, radius_rigid_robot_burger, test_point_coord, test_point_angle, rpm1, rpm2)

for action in action_set:
    X1 = rpm_1_1(clearance, radius_rigid_robot, test_point_coord, test_point_angle, action[0], action[1])  # (0,0,45) hypothetical start configuration
    print(X1)
    # for action in action_set:
    #     #(Xi, Yi, Thetai, UL, UR)
    #     X2 = rpm_1_1((clearance, radius_rigid_robot_burger, (X1[0], X1[1]), X1[2], action[0], action[1])

#
# plt.grid()
#
# ax.set_aspect('equal')
#
# plt.xlim(-5.5, 5.5)
# plt.ylim(-5.5, 5.5)
#
# plt.title('How to plot a vector in matplotlib ?', fontsize=10)
#
# plt.show()
# plt.close()



