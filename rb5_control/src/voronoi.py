#!/usr/bin/env python

from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
from math import *
import matplotlib.pyplot as plt
from heapq import heappop, heappush

actions = [(0, -1), (0, 1), (-1, 0), (1, 0), \
           (-1, -1), (-1, 1), (1, -1), (1, 1)]

class voronoi():

    def __init__(
        self, 
        map, 
        start, 
        goal, 
        tolerance, 
        max_iter = 5000, 
        verbose = False
    ):
        self.map = map
        self.start = start
        self.goal = goal
        self.tolerance = tolerance
        self.max_iter = max_iter
        self.verbose = verbose
        self.build_voronoi()
    
    def check_obstacle(self, row, col):
        return self.map[row, col] == 1
    
    def check_freespace(self, row, col):
        return not self.check_obstacle(row, col)
    
    def build_voronoi(self):
        h, w = self.map.shape
        coordinates = set()
        for row in range(1, h - 1):
            for col in range(1, w - 1):
                if self.check_obstacle(row, col):
                    if self.check_freespace(row - 1, col):
                        coordinates.add((col, row - 1))
                    if self.check_freespace(row + 1, col):
                        coordinates.add((col, row + 1))
                    if self.check_freespace(row, col - 1):
                        coordinates.add((col - 1, row))
                    if self.check_freespace(row, col + 1):
                        coordinates.add((col + 1, row))
        # print (coordinates)
        coordinates = np.array(list(coordinates)).reshape((-1, 2))
        self.voronoi = Voronoi(coordinates)
        if self.verbose:
            fig = voronoi_plot_2d(self.voronoi)
            fig.set_size_inches(16, 16)
            fig.suptitle("Visualization of the Voronoi Graph")
            plt.show()
    
    def cal_dist(self, x, y):
        return max(abs(x[0] - y[0]), abs(x[1] - y[1]))
    
    def update_curr_pos(self, curr_pos, action):
        '''Update curr_pos based on action.'''
        return [curr_pos[0] + action[0], curr_pos[1] + action[1]]
    
    def plan_path(self):
        vertices = self.voronoi.vertices.astype(int)
        nodes = []
        
        for vertice in vertices:
            x, y = vertice
            # if x == self.goal[0] and y == self.goal[1]:
            #     print ('Found start')
            if self.check_freespace(x, y):
                nodes.append(list(vertice))
                
        openset, closedset = [], []
        heappush(openset, (0, (self.start, [(0, 0)], 0)))
        token = 0

        while openset and token <= self.max_iter:
            curr_pos, path, cost = heappop(openset)[1]
            if curr_pos in closedset:
                continue
            closedset.append(curr_pos)
            if self.cal_dist(curr_pos, self.goal) <= self.tolerance:
                print ('[INFO] Voronoi Found Goal')
                return path
            for action in actions:
                update_pos = self.update_curr_pos(curr_pos, action)
                if update_pos in nodes:
                    update_dist = self.cal_dist(update_pos, self.goal)
                    update_cost = cost + update_dist
                    heappush(openset, (update_cost, (update_pos, path + [action], cost + 1)))
                    
            token += 1
            


        
                    

