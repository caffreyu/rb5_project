#!/usr/bin/env python

from map_constants import *
import numpy as np
import matplotlib.pyplot as plt
from a_star import aStar
import sys
from copy import deepcopy
import pickle as pk

# Add verbose=true or true to the command line to see the plots and waypoints
cmd = sys.argv
verbose = False
if len(cmd) > 1: 
    if 'true' in cmd[1] or 'True' in cmd[1]:
        verbose = True

# Get height, width of the map
height, width = 0, 0
for _, value in dict_outside_lm.items():
    x, y = value
    width = max(width, x)
    height = max(height, y)
arr_h, arr_w = int(height / cell_size) + 1, int(width / cell_size) + 1

# Build the initial map
map = np.zeros((arr_h, arr_w))

def ground_position_transform(ground_pos):
    '''Transform from ground position to map position.'''
    x, y = ground_pos
    return (height - y, x)

def array_position_transform(map_pos):
    '''Transform from map position to array position/index.'''
    x, y = map_pos
    row, col = int(x / cell_size), int(y / cell_size)
    return (row, col)

def ground_to_array_transform(ground_pos):
    '''Transform from ground position to array index.'''
    return array_position_transform(
        ground_position_transform(
            ground_pos
        )
    )

# Process obstacles
obs_lm1 = ground_to_array_transform(dict_inside_lm['lm1'])
obs_lm2 = ground_to_array_transform(dict_inside_lm['lm2'])
obs_lm3 = ground_to_array_transform(dict_inside_lm['lm3'])
obs_lm4 = ground_to_array_transform(dict_inside_lm['lm4'])
min_x = min(obs_lm1[0], obs_lm2[0], obs_lm3[0], obs_lm4[0])
min_y = min(obs_lm1[1], obs_lm2[1], obs_lm3[1], obs_lm4[1])
max_x = max(obs_lm1[0], obs_lm2[0], obs_lm3[0], obs_lm4[0])
max_y = max(obs_lm1[1], obs_lm2[1], obs_lm3[1], obs_lm4[1])

def can_add_obs(arr_pos):
    '''Checking if arr_pos can be treated as an obstacle.'''
    x, y = arr_pos
    return 0 <= x < arr_h and 0 <= y < arr_w

def add_obs(min_x, min_y, max_x, max_y):
    '''Add obstacle area to the mapping array.'''
    for x in range(min_x - 1, max_x + 2):
        for y in range(min_y - 1, max_y + 2):
            if can_add_obs(arr_pos = (x, y)):
                map[x, y] = 1

add_obs(min_x, min_y, max_x, max_y)

# Add walls
map[:, 0] = map[:, -1] = map[0, :] = map[-1, :] = 1

# Show the map
if verbose:
    plt.imshow(map)
    plt.show()

# Test A* path planner
arr_start = ground_to_array_transform(start)
arr_goal = ground_to_array_transform(goal)
a_star = aStar(
    start = arr_start,
    goal = arr_goal,
    tolerance = tolerance,
    map = map,
)

a_star.plan_path()

# Show the path
if verbose:
    map_disp = deepcopy(map)
    disp_start = deepcopy(arr_start)
    map_disp[arr_start[0], arr_start[1]] = 3
    for x, y in a_star.path:
        x_prev, y_prev = disp_start
        disp_start = [x_prev + x, y_prev + y]
        map_disp[x_prev + x, y_prev + y] = 2
    map_disp[arr_goal[0], arr_goal[1]] = 4

    plt.imshow(map_disp)
    plt.show()

# Generate waypoints
def array_to_ground_transform(arr_pos):
    '''Transform array index to ground position.'''
    x, y = arr_pos
    # Kinda cheat here because I used 3 - x * cell_size instead of height
    return [y * cell_size, 3 - x * cell_size]

arr_path = a_star.path
arr_track = list(arr_start)
prev_action = arr_path.pop(0)
arr_track[0] += prev_action[0]
arr_track[1] += prev_action[1]
waypoints = []

while arr_path:
    curr_action = arr_path.pop(0)
    delta_x, delta_y = curr_action
    x, y = arr_track
    if curr_action != prev_action:
        waypoints.append(array_to_ground_transform([x, y]))
    arr_track = [x + delta_x, y + delta_y]
    prev_action = curr_action

waypoints.append(array_to_ground_transform(arr_goal))

if verbose:
    print (np.array(waypoints))

# Store waypoints in a pickle file
with open('homework4_waypoints.pickle', 'wb') as f:
    pk.dump(np.array(waypoints), f)
