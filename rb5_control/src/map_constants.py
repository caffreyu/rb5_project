#!/usr/bin/env python

'''
Fill this file for
    - Landmark positions
    - Start and stop positions
    - Cell size
    - Robot clearance size
    - Tolerance for converge
'''

dict_outside_lm = {}
dict_inside_lm  = {}

dict_outside_lm['lm1'] = [3.05, 2.81] # TagID 4
dict_outside_lm['lm2'] = [0.24, 3.04] # TagID 5
dict_outside_lm['lm3'] = [2.80, 0.0] # TagID 8
dict_outside_lm['lm4'] = [3.06, 0.30] # Tag ID 7
dict_outside_lm['lm5'] = [0.22, 0.0] # TagID 1 
dict_outside_lm['lm6'] = [0.0, 0.22] # TagID 0
dict_outside_lm['lm7'] = [0.0, 2.82] # # TagID 2
dict_outside_lm['lm8'] = [2.85, 3.05] # TagID 3

dict_inside_lm['lm1'] = [1.53, 1.34] # TagID 4
dict_inside_lm['lm2'] = [1.755, 1.505] # TagID 5
dict_inside_lm['lm3'] = [1.305, 1.505] # TagID 8
dict_inside_lm['lm4'] = [1.53, 1.67] # TagID 7

start = [0.61, 0.61]
goal  = [2.44, 2.44]

cell_size = 0.1
robot_clearance = 0.2
tolerance = 2


