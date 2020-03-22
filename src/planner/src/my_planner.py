#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from const import *
from math import *
import copy
import argparse
from scipy import ndimage
from pdb import set_trace

from base_planner import Planner as BasePlanner, dump_action_table, ROBOT_SIZE

EIGHT_DIRECTION_ACTIONS = {
    "N": (0, 1),
    "S": (0, -1),
    "E": (1, 0),
    "W": (-1, 0),
    "NE": (1, 1),
    "SE": (-1, 1),
    "SW": (-1, -1),
    "NW": (1, -1),
}
FOUR_DIRECTION_ACTIONS = {
    "N": (0, -1),
    "S": (0, 1),
    "E": (1, 0),
    "W": (-1, 0),
}

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class Planner(BasePlanner):

    def map_callback(self):
        """Get the occupancy grid and inflate the obstacle by some pixels. You should implement the obstacle inflation yourself to handle uncertainty.
        """
        # Tuple = (-1, 100, ...)
        self.map = rospy.wait_for_message('/map', OccupancyGrid).data
        # self.map = tuple(np.loadtxt('map.txt'))

        aug_map = np.reshape(np.array(self.map), (self.world_width, self.world_height))
        aug_map = np.where(aug_map == 100, 1, aug_map)
        aug_map = np.where(aug_map == -1, 0, aug_map)
        # aug_map = np.flipud(aug_map)
        inflated_length = np.int(self.inflation_ratio + 2 * ROBOT_SIZE / self.resolution)
        aug_map = ndimage.grey_dilation(aug_map, size=(inflated_length, inflated_length))
        aug_map = np.where(aug_map == 1, 100, aug_map)
        aug_map = np.where(aug_map == 0, -1, aug_map)
        self.aug_map = aug_map
        # TODO: FILL ME! implement obstacle inflation function and define self.aug_map = new_mask

        # you should inflate the map to get self.aug_map
        # self.aug_map = new_map

    #This function return the path of the search
    def return_path(self, current_node):
        path = []
        current = current_node
        while current is not None:
            path.append(current.position)
            current = current.parent

        return path[::-1] # Return reversed path

    def astar_path(self, maze, start, end, cost=1):
        """
            Returns a list of tuples as a path from the given start to the given end in the given maze
            :param maze:
            :param cost
            :param start:
            :param end:
            :return:
        """

        # Create start and end node with initized values for g, h and f
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both yet_to_visit and visited list
        # in this list we will put all node that are yet_to_visit for exploration.
        # From here we will find the lowest cost node to expand next
        yet_to_visit_list = []
        # in this list we will put all node those already explored so that we don't explore it again
        visited_list = []

        # Add the start node
        yet_to_visit_list.append(start_node)

        # Adding a stop condition. This is to avoid any infinite loop and stop
        # execution after some reasonable number of steps
        outer_iterations = 0
        max_iterations = (len(maze) // 2) ** 10


        """
            1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
            2) Check max iteration reached or not . Set a message and stop execution
            3) Remove the selected node from yet_to_visit list and add this node to visited list
            4) Perofmr Goal test and return the path else perform below steps
            5) For selected node find out all children (use move to find children)
                a) get the current postion for the selected node (this becomes parent node for the children)
                b) check if a valid position exist (boundary will make few nodes invalid)
                c) if any node is a wall then ignore that
                d) add to valid children node list for the selected parent

                For all the children node
                    a) if child in visited list then ignore it and try next node
                    b) calculate child node g, h and f values
                    c) if child in yet_to_visit list then ignore it
                    d) else move the child to yet_to_visit list
        """
        #find maze has got how many rows and columns
        no_rows, no_columns = np.shape(maze)

        # Loop until you find the end
        while len(yet_to_visit_list) > 0:

            # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
            outer_iterations += 1

            # Get the current node
            current_node = yet_to_visit_list[0]
            current_index = 0
            for index, item in enumerate(yet_to_visit_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # if we hit this point return the path such as it may be no solution or
            # computation cost is too high
            if outer_iterations > max_iterations:
                print ("giving up on pathfinding too many iterations")
                return self.return_path(current_node)

            # Pop current node out off yet_to_visit list, add to visited list
            yet_to_visit_list.pop(current_index)
            visited_list.append(current_node)

            # test if goal is reached or not, if yes then return the path
            if current_node == end_node:
                return self.return_path(current_node)

            # Generate children from all adjacent squares
            children = []

            for moves in FOUR_DIRECTION_ACTIONS.values():
                i = self.convert_length_to_number_of_pixels(moves[0])
                j = self.convert_length_to_number_of_pixels(moves[1])
                # Get node position
                node_position = (current_node.position[0] + i, current_node.position[1] + j)

                # Make sure within range (check if within maze boundary)
                if (node_position[0] > (no_columns -1) or
                    node_position[0] < 0 or
                    node_position[1] > (no_rows - 1) or
                    node_position[1] < 0):
                    continue

                # Make sure walkable terrain
                collided = False
                if (i != 0):
                    i_start = i
                    increment = -1 if i_start > 0 else 1
                    while (i_start != 0):
                        if (maze[node_position[1]][current_node.position[0] + i_start] != -1):
                            collided = True
                            break
                        i_start += increment
                elif (j != 0):
                    j_start = j
                    increment = -1 if j_start > 0 else 1
                    while (j_start != 0):
                        if (maze[current_node.position[1] + j_start][node_position[0]] != -1):
                            collided = True
                            break
                        j_start += increment

                # for i in range(x_new):
                #     # Make sure walkable terrain
                #     if (maze[node_position[1]][node_position[0]] != -1):
                #         continue

                if (collided):
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:
                # Child is on the visited list (search entire visited list)
                if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                    continue

                # Create the f, g, and h values
                child.g = current_node.g + cost
                ## Heuristic costs calculated here, this is using eucledian distance
                child.h = (((child.position[0] - end_node.position[0]) ** 2) +
                        ((child.position[1] - end_node.position[1]) ** 2))

                child.f = child.g + child.h

                # Child is already in the yet_to_visit list and g cost is already lower
                if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                    continue

                # Add the child to the yet_to_visit list
                yet_to_visit_list.append(child)

    def convert_position_to_stage_map_coordinate(self, x, y):
        x_new = int(x / self.resolution)
        y_new = int(y / self.resolution)
        # x_new = int(x / self.resolution + self.inflation_ratio + np.ceil(ROBOT_SIZE / self.resolution) + 1)
        # y_new = int(y / self.resolution + self.inflation_ratio + np.ceil(ROBOT_SIZE / self.resolution) + 1)
        return x_new, y_new

    def convert_length_to_number_of_pixels(self, i):
        if i == 0:
            return i
        return int(i / self.resolution)
        # if i > 0:
        #     return int(i / self.resolution + self.inflation_ratio + np.ceil(ROBOT_SIZE / self.resolution) + 1)
        # return int(i / self.resolution - self.inflation_ratio - np.ceil(ROBOT_SIZE / self.resolution) - 1)

    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """

        # direction: theta: phi (E, 0, 0), (N, 90, 1), (W, 180, 2), (S, 270, -1)
        # x_start, y_start, phi = (1, 1, 0)
        # x_goal, y_goal = (5, 5)
        x_start, y_start, phi = self.get_current_discrete_state() # (1, 1, 0) []>
        x_goal, y_goal = self._get_goal_position()
        start = self.convert_position_to_stage_map_coordinate(x_start, y_start)
        goal = self.convert_position_to_stage_map_coordinate(x_goal, y_goal)
        path = self.astar_path(self.aug_map, start, goal)

        actions = []
        current_x, current_y = start
        current_phi = phi
        increment = self.convert_length_to_number_of_pixels(1)

        for new_position in path:
            if (current_x, current_y) == new_position:
                continue
            for next_phi in [0, 1, 2, -1]:
                # check next position after moving forward
                next_x = current_x
                next_y = current_y
                if next_phi == 0 or next_phi == 2:
                    next_x += (next_phi * (-increment) + increment)
                elif next_phi == -1 or next_phi == 1:
                    next_y += (next_phi * increment)

                # if moving forward goes to the correct new position
                if (next_x, next_y) == new_position:
                    phi_diff = next_phi - current_phi
                    # Perform rotate 180
                    if abs(phi_diff) == 2:
                        actions.append((0, 1))
                        actions.append((0, 1))
                    # Perform turn right
                    elif phi_diff % 4 == 1:
                        actions.append((0, 1))
                    # Perform turn left
                    elif phi_diff % 4 == 3:
                        actions.append((0, -1))
                    # Perform moving forward
                    actions.append((1, 0))
                    # set the current position with new one
                    current_x, current_y, current_phi = next_x, next_y, next_phi
                    break

        self.action_seq = actions

    def collision_checker(self, x, y):
        """TODO: FILL ME!
        You should implement the collision checker.
        Hint: you should consider the augmented map and the world size

        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
        
        Returns:
            bool -- True for collision, False for non-collision
        """
        x_stage, y_stage = self.convert_position_to_stage_map_coordinate(x, y)

        return self.aug_map[y_stage][x_stage] == 100


if __name__ == "__main__":
    # TODO: You can run the code using the code below
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', type=str, default='1,8',
                        help='goal position')
    parser.add_argument('--com', type=int, default=0,
                        help="if the map is com1 map")
    args = parser.parse_args()

    try:
        goal = [int(pose) for pose in args.goal.split(',')]
    except:
        raise ValueError("Please enter correct goal format")

    if args.com:
        width = 2500
        height = 983
        resolution = 0.02
    else:
        width = 200
        height = 200
        resolution = 0.05

    # TODO: You should change this value accordingly
    inflation_ratio = 3
    planner = Planner(width, height, resolution, inflation_ratio=inflation_ratio)
    planner.set_goal(goal[0], goal[1])
    if planner.goal is not None:
        planner.generate_plan()

    # You could replace this with other control publishers
    planner.publish_discrete_control()

    # save your action sequence
    result = np.array(planner.action_seq)
    # np.savetxt("actions_continuous.txt", result, fmt="%.2e")

    # for MDP, please dump your policy table into a json file
    # dump_action_table(planner.action_table, 'mdp_policy.json')

    # spin the ros
    rospy.spin()
