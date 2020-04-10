import matplotlib.pyplot as plt
import numpy as np

# # plot the path:
fig, ax = plt.subplots()
plt.grid()
# #
ax.set_aspect('equal')

clearance = 0.1
radius_rigid_robot = 0.1
augment_distance = radius_rigid_robot + clearance

# ax.spines['top'].set_visible(False)
# ax.spines['right'].set_visible(False)

# ax.set_xlabel('sample')
# ax.set_ylabel('z-score')

# plt.axhline(0, color='black')



circle1 = plt.Circle((2, 3), radius=1 + augment_distance)
ax.add_patch(circle1)

circle2 = plt.Circle((0, 0), radius=1 + augment_distance)
ax.add_patch(circle2)

circle3 = plt.Circle((-2, -3), radius=1 + augment_distance)
ax.add_patch(circle3)

circle4 = plt.Circle((2, -3), radius=1 + augment_distance)
ax.add_patch(circle4)

points = [[-1.25+augment_distance, 3.75+ augment_distance], [-1.25+augment_distance, 2.25- augment_distance], [-2.75-augment_distance, 2.25- augment_distance], [-2.75-augment_distance, 3.75+ augment_distance]]
square1 = plt.Polygon(points)
ax.add_patch(square1)

points = [[-3.25+augment_distance, 0.75+augment_distance], [-4.75-augment_distance, 0.75+augment_distance], [-4.75-augment_distance, -0.75-augment_distance], [-3.25+augment_distance, -0.75-augment_distance]]
square2 = plt.Polygon(points)
ax.add_patch(square2)

points = [[3.25-augment_distance, 0.75+augment_distance], [4.75+augment_distance, 0.75+augment_distance],[4.75+augment_distance, -0.75-augment_distance],[3.25-augment_distance, -0.75-augment_distance]]
square3 = plt.Polygon(points)
ax.add_patch(square3)

# 0.1 = thickness of the boundary of the world map
# add the left boundary
points = [[-5.1, -5.1], [-5.1, 5.1],
          [-5.1 + augment_distance + 0.1, 5.1], [-5.1 + augment_distance + 0.1, -5.1]]
polygon = plt.Polygon(points)
ax.add_patch(polygon)

# add the top boundary
points = [[5.1, 5.1], [-5.1, 5.1], [-5.1, 5.1 - augment_distance - 0.1],
          [5.1, 5.1 - augment_distance - 0.1]]
polygon = plt.Polygon(points)
ax.add_patch(polygon)

# add the lower boundary
points = [[5.1, -5.1], [-5.1, -5.1], [-5.1, -5.1 + augment_distance + 0.1],
          [5.1, -5.1 + augment_distance + 0.1]]
polygon = plt.Polygon(points)
ax.add_patch(polygon)

# add the right boundary
points = [[5.1, 5.1], [5.1, -5.1], [5.1 - augment_distance - 0.1, -5.1],
          [5.1 - augment_distance - 0.1, 5.1]]
polygon = plt.Polygon(points)
ax.add_patch(polygon)

plt.xlim(-5.5, 5.5)
plt.ylim(-5.5, 5.5)

plt.show()