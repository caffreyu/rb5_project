#!/usr/bin/env python

import numpy as np
from math import sqrt
from heapq import heappop, heappush

actions = [(0, -1), (0, 1), (-1, 0), (1, 0), \
           (-1, -1), (-1, 1), (1, -1), (1, 1)]

# only class names are in camal case
class aStar():
    '''Class for A* path planner.'''

    def __init__(
        self, 
        start, 
        goal, 
        tolerance, 
        map,
        max_iter = 5000, 
    ):
        '''Initialize the aStar class.'''
        self.start = start
        self.goal = goal
        self.tolerance = tolerance
        self.map = map
        self.map_width, self.map_height = np.shape(self.map)
        self.max_iter = max_iter
        self.visited = []
        self.open_set = []
        self.path = []
        heappush(
            self.open_set, 
            (self.cal_manhanttan_dist(start, goal), (start, [], 0))
        )
    

    def get_type_name(self, type_string):
        '''Get the exact type from the python type string.'''
        type_string_list = type_string.split()
        type_string_processed = type_string_list[1]
        length = len(type_string_processed)
        return type_string_processed[1 : length - 2]

    
    def cal_manhanttan_dist(self, point_a, point_b):
        '''Calculate manhanttan distance between point_a and point_b.'''
        assert len(point_a) == len(point_b) == 2, \
            "Inputs for aStar.cal_manhanttan_dist() are invalid"
        x_a, y_a = point_a
        x_b, y_b = point_b
        return max(abs(x_a - x_b), abs(y_a - y_b))


    def cal_euclidean_dist(self, point_a, point_b):
        '''Calculate manhanttan distance between point_a and point_b.'''
        assert len(point_a) == len(point_b) == 2, \
            "Inputs for aStar.cal_euclidean_dist() are invalid"
        x_a, y_a = point_a
        x_b, y_b = point_b
        return sqrt(pow(x_a - x_b, 2) + pow(y_a - y_b, 2))

    
    def check_goal(self, curr_pos):
        '''Check if vehicle has reached the goal.'''
        return self.cal_euclidean_dist(curr_pos, self.goal) <= self.tolerance
    

    def update_curr_pos(self, curr_pos, action):
        '''Update curr_pos based on action.'''
        return (curr_pos[0] + action[0], curr_pos[1] + action[1])
    

    def check_map(self, update_pos):
        '''Check map for non-diag movement.'''
        x, y = update_pos
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            return self.map[x, y] == 0
    

    def check_map_diag(self, curr_pos, action):
        '''Check map for diagonal movement.'''
        #? How we wanna define the check for diagonal movement? 
        curr_x, curr_y = curr_pos
        action_x, action_y = action
        update_x, update_y = curr_x + action_x, curr_y + action_y
        return (
            self.check_map([update_x, curr_y]) 
            and self.check_map([curr_x, update_y])
            and self.check_map([update_x, update_y])
        )
    
    
    def plan_path(self):
        '''Plan the path from start to goal.'''
        token = 0
        while token < self.max_iter:
            curr_pop = heappop(self.open_set)
            curr_pos, path, cost = curr_pop[1]
            # Check goal
            if self.check_goal(curr_pos):
                self.path = path
                print ("[INFO] A* Found Goal")
                break
            # Check visited
            if curr_pos in self.visited:
                continue
            self.visited.append(curr_pos)
            # Expand node
            for action in actions:
                update_pos = self.update_curr_pos(curr_pos, action)
                if abs(action[0]) == abs(action[1]):
                    if self.check_map_diag(curr_pos, action):
                        update_dist = self.cal_euclidean_dist(update_pos, self.goal)
                        update_cost = cost + update_dist
                        heappush(self.open_set, (update_cost, (update_pos, path + [action], cost + 1)))
                else:
                    if self.check_map(update_pos):
                        update_dist = self.cal_euclidean_dist(update_pos, self.goal)
                        update_cost = cost + update_dist
                        heappush(self.open_set, (update_cost, (update_pos, path + [action], cost + 1)))
            token += 1

if __name__ == '__main__':
    map = np.zeros((6, 6))
    map[2 : 4, 1 : 3] = np.ones((2, 2))
    map[4, 3] = map[5, 2] = map[1, 4] = 1
    print (map)

    start = (5, 0)
    goal  = (3, 3)
    tol = 0
    a_star = aStar(start = start, goal = goal, tolerance = tol, map = map)
    a_star.plan_path()
    print (a_star.path)
