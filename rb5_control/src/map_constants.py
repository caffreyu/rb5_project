#!/usr/bin/env python

'''
Fill this file for
    - Landmark positions
    - Start and stop positions
    - Cell size
    - Tolerance for converge
'''

# Below are sample data for testing
dict_outside_lm = {}
dict_inside_lm  = {}

dict_outside_lm['lm1'] = [0.93, 3.05]
dict_outside_lm['lm2'] = [0, 0.82]
dict_outside_lm['lm3'] = [3.05, 2.12]
dict_outside_lm['lm4'] = [2.14, 3.05]
dict_outside_lm['lm5'] = [3.05, 0.8]
dict_outside_lm['lm6'] = [2.45, 0]
dict_outside_lm['lm7'] = [1.07, 0]
dict_outside_lm['lm8'] = [0, 2.36]

dict_inside_lm['lm1'] = [1.75, 1.5]
dict_inside_lm['lm2'] = [1.5, 1.75]
dict_inside_lm['lm3'] = [1.5, 1.25]
dict_inside_lm['lm4'] = [1.25, 1.5]

start = [2.5, 0.5]
goal  = [0.5, 2.5]

cell_size = 0.1
tolerance = 0


