# ENPM 661 - Planning for Autonomous Robots
# Project 3 - Implementing A star search Algorithm for Rigid Robot
# Team - Nidhi Bhojak           , UID - 116787529
#        Kulbir Singh Ahluwalia , UID - 116836050

# import intersection_check as ic

from intersection_check import *
from obstacle_check import *
from matplotlib.patches import *
import numpy as np
import math
import matplotlib.pyplot as plt
from time import process_time
import cv2

visited = np.zeros((600,400,12))


class GraphNode:
    def __init__(self, point):
        self.position = point
        self.cost = math.inf
        self.total_cost = 0
        self.parent = None
        self.angle = 0

def heu(node1, node2):
  dist= math.sqrt( (node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
  return dist

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

    left_wheel_velocity = (r * (2 / 60) * pi * (rpm1))
    right_wheel_velocity = (r * (2 / 60) * pi * (rpm2))
    bot_velocity = (left_wheel_velocity + right_wheel_velocity) / 2

    distance_covered = bot_velocity * 1

    Thetan = 180 * (Thetan) / pi
    new_point = [Xn, Yn, Thetan]

    if (test_point_obstacle_check(clearance, radius_rigid_robot, new_point)) == False:
        return new_point, distance_covered
    else:
        return None, None


rpm1 = 40
rpm2 = 40
test_point_coord = (-3.5,-3)
test_point_angle = 30
r = 0.033
L = 0.160
pi = math.pi


def get_new_node(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm1, rpm2):
    action_map = {
        'F1': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm1, rpm1),
        'F2': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm2, rpm2),

        'UP1': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, 0, rpm1),
        'UP2': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, 0, rpm2),
        'UP3': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm2, rpm1),

        'DN1': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm1, 0),
        'DN2': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm2, 0),
        'DN3': rpm_to_new_point(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm1, rpm2),
    }
    return action_map[action]

action_set = [[rpm2, rpm1],
              [0,rpm1],
              [0,rpm2],
              [rpm1,rpm1],
              [rpm2,rpm2],
              [rpm1,0],
              [rpm2,0]
              [rpm1, rpm2]]





def get_minimum_element(queue):
    min_index = 0
    for index in range(len(queue)):
        if queue[index].total_cost < queue[min_index].total_cost:
            min_index = index

    #return the object with the lowest total _cost
    return queue.pop(min_index)

def round_off_till_threshold(number, threshold):
    number_double = round((number / threshold))
    return int(number_double*threshold)


def approximation(x, y, angle_theta):
    x = round_off_till_threshold(x, 0.5)
    y = round_off_till_threshold(y, 0.5)

    angle_theta = round_off_till_threshold(angle_theta, 30)
    return (x, y, int(angle_theta/30))


def check_goal(position, goal):
  if (((position[0]- goal[0])**2)+ ((position[1] - goal[1])**2)) <= (1.5)**2:
    return True
  else:
    return False


def reset_angle_in_range(angle):
      if angle >= 360:
          return angle % 360
      if angle < 0:
          return 360 + angle
      else:
          return angle

def find_path_astar(image, start_node_pos, goal_node_pos, clearance, radius_rigid_robot, step_size_robot, initial_angle):
    #(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm1, rpm1)
    # class GraphNode:
    #     def __init__(self, point):
    #         self.position = point
    #         self.cost = math.inf
    #         self.total_cost = 0
    #         self.parent = None
    #         self.angle = 0

    start_node = GraphNode(start_node_pos)
    start_node.cost = 0  # initialise cost of start node = 0
    start_node.angle = initial_angle #Set the angle of start node as given by user

    # visited = list()  # list of all visited nodes
    queue = [start_node]  # queue is a list that contains yet to be explored node "objects"
    # print(queue)

    actions = ["S", "UP1", "UP2", "DN1", "DN2"]  # define a list with all the possible actions
    visited_set = set()  # a set is used to remove the duplicate node values
    visited_list = []  # a list is used to visualize the order of nodes visited and to maintain order
    cost_updates_matrix = np.zeros((400, 600, 12), dtype=object)   #To keep track of duplicate nodes
    # note that the datatype in the above command is "object" because everything in python is an object, even numbers are objects.
    # print(cost_updates_matrix)

    cost_updates_matrix[:, :, :] = math.inf  # initialise cost update matrix with infinite costs for every node
    goal_reached = False  # set the goal_reached flag to zero initially
    parent_child_map = {}  # define a dictionary to store parent and child relations
    # key in a dict can't be a list, only immutable things can be used as keys, so use tuples as keys
    parent_child_map[tuple([start_node_pos[0], start_node_pos[1], initial_angle])] = None  # for start node, there is no parent
    # parent_child_map[key=tuple(x,y,theta)] = value of the key-value pair of the dictionary = None as node 1 has no parent :(
    # print(parent_child_map)

    start = process_time()  # start the time counter for calculating run time for the A* search algorithm
    last_node = None    #called last node because goal is not reached as such, we only reach in a 1.5 radius circle around the goal point

    while len(queue) > 0:  # as long as there are nodes yet to be checked in the queue, while loop keeps running
        current_node = get_minimum_element(queue)  # choose the node object with minimum cost
        current_point = current_node.position  # store the position from the (minimum cost) current_node in "current_point"
        current_angle = current_node.angle
        # visited.append(str(current_point))  # convert the current_point to an immutable string and store it in the list "visited"


        if approximation(current_point[0], current_point[1], current_angle) not in visited_set:     #to avoid adding duplicates in the list
            visited_list.append(current_node)
        visited_set.add(approximation(current_point[0], current_point[1], current_angle))  # you can only put immutable objects in a set, string is also immutable


        if check_goal(current_point, goal_node_pos):
            goal_reached = True
            print('Goal Reached')
            print("Cost = ", current_node.cost)
            last_node = current_node
            break

        # to generate, explore and append possible next positions, make a list of all the generated child nodes
        child_nodes = []
        # actions = ["S", "UP1", "UP2", "DN1", "DN2"] ,     action = iterable element
        for action in actions:
            # get_new_node is run for every action , U, D, L, R, UR, DR, UL, DL
            new_point, base_cost = get_new_node(clearance, radius_rigid_robot, test_point_coord, test_point_angle, rpm1, rpm1)
            print(new_point)
            if new_point is not None:  # present in the explorable area and not in obstacle
                #we get a None value as return "None" only when the new_point is in obstacle
                # print(len(new_point))
              #if new_point is not visited[int()][][]
                child_nodes.append((new_point, base_cost))  # append the new node in child nodes along with cost

        # print(child_nodes[0])        #first element of the list = ([x,y,theta],cost)
        # print(child_nodes[0][0])     #first element of first element of the list = [x,y,theta]
        # print(child_nodes[0][0][1])  #second element of first element of first element of the list = y

        for child in child_nodes:  # child = iterable element in child_nodes = of the format ([x,y,angle],cost)
            child[0][2] = reset_angle_in_range(child[0][2])
            approx_x, approx_y, approx_theta = approximation(child[0][0], child[0][1], child[0][2])

            # duplicate node removal
            if approximation(child[0][0], child[0][1], child[0][2]) not in visited_set:
                child_position = child[0]  # child[0] = [x,y, theta]
                child_position_x = child[0][0]  # child[0][0] = x
                child_position_y = child[0][1]  # child[0][1] = y
                child_position_theta = child[0][2]  # child[0][2] = theta
                print(approx_x, approx_y, approx_theta)

                # to find the previous cost from the cost_update_matrix
                prev_cost = cost_updates_matrix[approx_y, approx_x, approx_theta]  # row,column
                # prev_cost = cost_updates_matrix[y, x]  # row,column

                # add the cost of the child to the current node's cost to get new cost
                new_cost = child[1] + current_node.cost  # child[1] = cost
                total_cost = new_cost + heu(child[0], goal_node_pos)

                if total_cost < prev_cost:
                    print(approx_x, approx_y, approx_theta)

                    #in cost_updates_matrix, put new cost in place of infinity
                    cost_updates_matrix[approx_y, approx_x, approx_theta] = new_cost
                    child_node = GraphNode(child[0])
                    child_node.cost = new_cost
                    child_node.parent = current_node
                    child_node.total_cost = total_cost
                    child_node.angle = child[0][2]
                    queue.append(child_node)  # child_node is yet to be explored
                    #
                    #key=child coord,  value=parent coord
                    parent_child_map[tuple(child[0])] = tuple([current_point[0], current_point[1], current_angle])  # key, always immutable, here, tuple = tuple(child[0])
                    #   #value, can be anything = current_point

    end = process_time()
    print("Time to completion:", (end - start))

    if goal_reached:
        print('Reached')
        return visited_list, parent_child_map, last_node
        #return visited_set, parent_child_map, last_node
    else:
        #if goal is not found then the visited list is returned as None, parent child, Last node map are also returned as None
        return None, None, None


def check_inputs_wrt_obstacles(start_node_x, start_node_y, goal_node_x, goal_node_y):

    if test_point_obstacle_check(clearance, radius_rigid_robot, [start_node_x, start_node_y], None):
        print("Start node is inside an obstacle. Enter some other coordinates. Restart program!")
        exit(0)

    if test_point_obstacle_check(clearance, radius_rigid_robot, [goal_node_x, goal_node_y], None):
        print("Goal node is inside an obstacle. Enter some other coordinates. Restart program!")
        exit(0)


# # Uncomment to choose different positions:-
# start_node_x = int(input("Enter the starting x coordinate for the rigid robot\n"))
# start_node_y = int(input("Enter the starting y coordinate for the rigid robot\n"))
# initial_angle = int(input("Enter the initial angle of the robot in degree\n"))
#
# goal_node_x = int(input("Enter the goal x coordinate for the rigid robot\n"))
# goal_node_y = int(input("Enter the goal y coordinate for the rigid robot\n"))

# #two rpm values
# rpm1 = int(input("Enter the rpm value for the left wheel\n"))
# rpm2 = int(input("Enter the rpm value for the right wheel\n"))


# radius_rigid_robot = int(input("Enter the radius of the rigid robot \n"))
# clearance = int(input("Enter the desired clearance for the rigid robot\n"))

turtlebot_diameter = 0.21 #0.21 metres
wheel_distance = 0.16


# step_size_robot = int(input("Enter the step size of movement of the robot: "))
# if (step_size_robot < 1 and step_size_robot > 10):
#     print("Step_size_robot is out of range. Enter step_size_robot from [0,10]. Restart program!")
#     exit(0)
#
#

# for testing
start_node_x = 1.5
start_node_y = 1.5

goal_node_x = 4.5
goal_node_y = 4.5

step_size_robot = 0.1
initial_angle = 60

clearance = 0.2
radius_rigid_robot = 0.105
augment_distance = radius_rigid_robot + clearance

if (start_node_x < -5.1 and start_node_x > 5.1) and (goal_node_x < -5.1 and goal_node_x > 5.1):
    print("X coordinate is out of range. Enter x from [0,300]. Restart program!")
    exit(0)

if (start_node_y < -5.1 and start_node_y > 5.1) and (goal_node_y < -5.1 and goal_node_y > 5.1):
    print("Y coordinate is out of range. Enter y from [0,200]. Restart program!")
    exit(0)

check_inputs_wrt_obstacles(start_node_x, start_node_y, goal_node_x, goal_node_y)


def main():

    visited_list, parent_child_map, last_node = find_path_astar(None, start_node_position, goal_node_position, clearance,
                                                        radius_rigid_robot, step_size_robot, initial_angle)

    # # plot the path:
    fig, ax = plt.subplots()
    plt.grid()
    # #
    ax.set_aspect('equal')

    circle1 = plt.Circle((2, 3), radius=1 + augment_distance)
    ax.add_patch(circle1)

    circle2 = plt.Circle((0, 0), radius=1 + augment_distance)
    ax.add_patch(circle2)

    circle3 = plt.Circle((-2, -3), radius=1 + augment_distance)
    ax.add_patch(circle3)

    circle4 = plt.Circle((2, -3), radius=1 + augment_distance)
    ax.add_patch(circle4)

    points = [[-1.25 + augment_distance, 3.75 + augment_distance], [-1.25 + augment_distance, 2.25 - augment_distance],
              [-2.75 - augment_distance, 2.25 - augment_distance], [-2.75 - augment_distance, 3.75 + augment_distance]]
    square1 = plt.Polygon(points)
    ax.add_patch(square1)

    points = [[-3.25 + augment_distance, 0.75 + augment_distance], [-4.75 - augment_distance, 0.75 + augment_distance],
              [-4.75 - augment_distance, -0.75 - augment_distance],
              [-3.25 + augment_distance, -0.75 - augment_distance]]
    square2 = plt.Polygon(points)
    ax.add_patch(square2)

    points = [[3.25 - augment_distance, 0.75 + augment_distance], [4.75 + augment_distance, 0.75 + augment_distance],
              [4.75 + augment_distance, -0.75 - augment_distance], [3.25 - augment_distance, -0.75 - augment_distance]]
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

    out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'XVID'), 1, (565, 379))

    #visited_list is none only when the goal is not found
    if visited_list is not None:

        # for loop is for visited_list aka explored nodes
        for ind, v in enumerate(visited_list):
            print(ind) #ind = index
            child_pos = v.position
            if v.parent is not None:
                parent_pos = v.parent.position
                ax.quiver(parent_pos[0], parent_pos[1], child_pos[0]-parent_pos[0], child_pos[1]-parent_pos[1], units='xy', scale=1)

                if ind%3000==0:
                    # "." denotes the current directory
                    # ".." denotes the previous directory
                    plt_name = './plots/plot' + str(ind) + '.png'

                    #savefig is a function of matplotlib
                    plt.savefig(plt_name, bbox_inches='tight')

                    #read the image stored using cv2
                    plot_img = cv2.imread(plt_name)

                    #plot_img.shape = gives dimension of the frame
                    #print('frame', plot_img.shape)

                    # write the image in the video
                    out.write(plot_img)

            #image[v[1], v[0]] = (255, 255, 0)
            #resized_new = cv2.resize(image, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)
            #cv2_imshow(resized_new)
            #if cv2.waitKey(1) == 27:
            #    break



        trace_path = []

        # last_node = child of some node
        # now we need to track back to the starting position and print the final path
        child_pos = last_node.position
        child_pos_tuple = (child_pos[0], child_pos[1], last_node.angle)

        #find the parent of last node
        parent = parent_child_map[child_pos_tuple]

        # to trackback and plot the vectors
        while parent is not None:
            # parent is None only at the first node
            #ax.quiver to plot the vector
            ax.quiver(parent[0], parent[1], child_pos_tuple[0] - parent[0], child_pos_tuple[1] - parent[1], units='xy', scale=1, color='g')

            #trace_path.append(parent)
            # Every parent has a child, kyunki saas bhi kabhi bahu thi... - Guruji
            # the current parent is made a child
            child_pos_tuple = parent

            # find the parent of the parent using the parent_child_map
            parent = parent_child_map[parent]


        plt_name = './plots/plot.png'
        plt.savefig(plt_name, bbox_inches='tight')
        plot_img = cv2.imread(plt_name)
        out.write(plot_img)

        out.release()


        fig.show()
        # fig.draw()
        plt.show()
    else:
        print('Cannot find goal.')

    '''
    trace_path.reverse()
    for point in trace_path:
        image[point[1], point[0]] = (200, 0, 200)
        resized_new = cv2.resize(image, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)

    cv2_imshow(resized_new)

    print("Press any key to Quit")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    plt.imshow(image)
    # print(image)
    plt.show()
    '''


if __name__ == "__main__":
    main()


