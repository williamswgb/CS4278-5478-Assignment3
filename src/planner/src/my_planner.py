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
        # self.map = rospy.wait_for_message('/map', OccupancyGrid).data
        self.map = tuple(np.loadtxt('map.txt'))

        aug_map = np.reshape(np.array(self.map), (self.world_height, self.world_width))
        aug_map = np.where(aug_map == 100, 1, aug_map)
        aug_map = np.where(aug_map == -1, 0, aug_map)
        # aug_map = np.flipud(aug_map)
        inflated_length = np.int(self.inflation_ratio + 2 * ROBOT_SIZE / self.resolution)
        aug_map = ndimage.grey_dilation(aug_map, size=(inflated_length, inflated_length))
        aug_map = np.where(aug_map == 1, 100, aug_map)
        aug_map = np.where(aug_map == 0, -1, aug_map)

        # TODO: FILL ME! implement obstacle inflation function and define self.aug_map = new_mask
        # you should inflate the map to get self.aug_map
        self.aug_map = aug_map

    def astar_path(self, start, end, cost=1):
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
        # Loop until you find the end
        while len(yet_to_visit_list) > 0:
            # Get the current node
            current_node = yet_to_visit_list[0]
            current_index = 0
            for index, item in enumerate(yet_to_visit_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current node out off yet_to_visit list, add to visited list
            yet_to_visit_list.pop(current_index)
            visited_list.append(current_node)

            # test if goal is reached or not, if yes then return the path
            if self._check_goal(current_node.position):
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent

                return path[::-1] # Return reversed path

            # Generate children from all adjacent squares
            children = []
            x, y, theta = current_node.position
            for w in [0, 1, 3]:
                v = 1 if w == 0 else 0
                new_position = self.discrete_motion_predict(x, y, theta, v, w)
                if (new_position):
                    new_node = Node(current_node, new_position)
                    children.append(new_node)

            # Loop through children
            for child in children:
                # Child is on the visited list (search entire visited list)
                if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                    continue

                # Create the f, g, and h values
                child.g = current_node.g + cost
                ## Heuristic costs calculated here, this is using eucledian distance
                child.h = self._d_from_goal(current_node.position)

                child.f = child.g + child.h

                # Child is already in the yet_to_visit list and g cost is already lower
                if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                    continue

                # Add the child to the yet_to_visit list
                yet_to_visit_list.append(child)

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
        start = (1, 1, 0)
        # start = self.get_current_discrete_state() # (1, 1, 0) []>
        goal = self._get_goal_position()
        path = self.astar_path(start, goal)
        actions = []
        if path is not None:
            for i in range(1, len(path)):
                x_prev, y_prev, theta_prev = path[i-1]
                x_cur, y_cur, theta_cur = path[i]
                r = int(sqrt((x_cur - x_prev)**2 + (y_cur - y_prev)**2))
                w = int(theta_cur - theta_prev)
                w = w if w < 3 else w % -4
                actions.append((r, w))
        else:
            print("No path found")

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
        x_map, y_map = int(floor(x / self.resolution)), int(floor(y / self.resolution))
        is_collided = (x_map > (self.world_width -1) or x_map < 0 or
          y_map > (self.world_height - 1) or y_map < 0 or
          self.aug_map[y_map][x_map] == 100)

        return is_collided


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
        inflation_ratio = 5
    else:
        width = 200
        height = 200
        resolution = 0.05
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
