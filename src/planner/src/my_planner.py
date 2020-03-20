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
from pdb import set_trace

from base_planner import Planner as BasePlanner, dump_action_table

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
    "N": (0, 1),
    "S": (0, -1),
    "E": (1, 0),
    "W": (-1, 0),
}

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None, direction=None):
        self.parent = parent
        self.position = position
        self.direction = direction

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

        # Convert 1-D tuple map into 2-D tuple map based on width and height
        # new_map = np.reshape(np.array(self.map), (self.world_width, self.world_height))
        # new_map = np.array([
        #     [-1, -1, -1, -1, 100, -1, -1, -1, -1, G],
        #     [-1, S, -1, -1, 100, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, 100, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, 100, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, 100, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, 100, -1, -1, -1, -1, -1]
        # ])
        # self.world_height = 10
        # self.world_width = 10
        # self.inflation_ratio = 1

        for i in range(self.world_height):
            for j in range(self.world_width):
                if (new_map[i][j] == 100):
                    for k in range(1, self.inflation_ratio+1):
                        # inflate top
                        top_i = i - k
                        if top_i >= 0 and new_map[top_i][j] == -1:
                            new_map[top_i][j] = 50
                        # inflate bottom
                        bottom_i = i + k
                        if bottom_i < self.world_height and new_map[bottom_i][j] == -1:
                            new_map[bottom_i][j] = 50
                        # inflate left
                        left_j = j - k
                        if left_j >= 0 and new_map[i][left_j] == -1:
                            new_map[i][left_j] = 50
                        # inflate right
                        right_j = j + k
                        if right_j < self.world_width and new_map[i][right_j] == -1:
                            new_map[i][right_j] = 50

        # convert new_map value from inflation value 50 to obstacle value 100
        new_map = np.where(new_map != 50, new_map, 100)
        new_map = tuple(new_map.flatten())
        # TODO: FILL ME! implement obstacle inflation function and define self.aug_map = new_mask

        # you should inflate the map to get self.aug_map
        self.aug_map = new_map

    def astar_path(self, maze, start, end):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                direction = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    if (current.direction is not None):
                        direction.append(current.direction)
                    current = current.parent

                return path[::-1] # Return reversed path

            # Generate children
            children = []
                        # Action    S        N       W       E        SW        SE       NW      NE
                        # 1,1 => (1,  0), (1, 2), (0,  1), (2, 1),  (0,  0),  (0, 2), (2,  0), (2, 2)
            for direction in FOUR_DIRECTION_ACTIONS.keys(): # Adjacent squares
                action = FOUR_DIRECTION_ACTIONS[direction]
                # Get node position
                new_position = (current_node.position[0] + action[0], current_node.position[1] + action[1])

                # Make sure within range
                if new_position[1] > (len(maze) - 1) or new_position[1] < 0 or new_position[0] > (len(maze[len(maze)-1]) -1) or new_position[0] < 0:
                    continue

                # Make sure walkable terrain
                if maze[new_position[1]][new_position[0]] != -1:
                    continue

                # Create new node
                new_node = Node(current_node, new_position, direction)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)

    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """
        # x, y, phi = (1, 1, 0)
        # map = np.array([
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, 100, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, 100, 100, 100, -1, -1, -1, -1],
        #     [-1, -1, 100, 100, 100, 100, 100, -1, -1, -1],
        #     [-1, -1, 100, 100, 100, 100, 100, -1, -1, -1],
        #     [-1, -1, -1, 100, 100, 100, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, 100, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        # ])
        # start = (x, y)
        # goal = (9, 9)
        # path = self.astar_path(map, start, goal)

        # direction: theta: phi (E, 0, 0), (N, 90, 1), (W, 180, 2), (S, 270, -1)
        x, y, phi = self.get_current_discrete_state() # (1, 1, 0) []>
        start = (x, y)
        goal = self._get_goal_position()
        path = self.astar_path(self.map, start, goal)

        actions = []
        current_x, current_y, current_phi = (x, y, phi)

        for new_position in path:
            if (current_x, current_y) == new_position:
                continue
            for next_phi in [0, 1, 2, -1]:
                # check next position after moving forward
                next_x = current_x
                next_y = current_y
                if next_phi == 0 or next_phi == 2:
                    next_x += (next_phi * -1 + 1)
                elif next_phi == -1 or next_phi == 1:
                    next_y += next_phi

                # if moving forward goes to the correct new position
                if (next_x, next_y) == new_position:
                    phi_diff = next_phi - current_phi
                    # Perform rotate 180
                    if abs(phi_diff) == 2:
                        actions.append((0, 1))
                        actions.append((0, 1))
                    # Perform turn left
                    elif phi_diff % 4 == 1:
                        actions.append((0, 1))
                    # Perform turn right
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
        aug_map = np.reshape(np.array(self.aug_map), (self.world_width, self.world_height))
        return aug_map[y][x] == 100


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
