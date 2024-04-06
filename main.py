# -*- coding: utf-8 -*-
"""
Created on Thu Mar 14 05:22:15 2024

@author: ESHANT
@efficiently edited by: THE GREAT DON
"""
from inspect import FrameInfo
from math import exp
import time
import numpy as np
import search 
import successor 
import matplotlib.pyplot as plt 
import map_grid
from matplotlib.colors import LinearSegmentedColormap
import pandas as pd



problem=successor.Problem()


#path= search.aStarSearch(problem, robot_id, start_state, planner, collison_point, neighbor_robot_point, goal_states)

start = [190,366, 90]
goal  = [ 650,150, 90]              #map check 300 220

actual_start = [start[0]+ 618 , start[1]+94, start[2] ]
actual_goal = [goal[0]+ 618 , goal[1]+94 , goal[2] ]
map_grid.map.start= start
map_grid.map.goal = goal

init_time = time.time()

# iter = 0
# while iter >=1000:
path,explored , fringe= search.aStarSearch(problem, 1, start , 1, [], [], [],iter)
iter = 10

fin_time = time.time()
time_calc = fin_time-init_time
print(time_calc,'time for planning')
print(len(path),'path length')
# print(path , 'path')
# print(fringe,'fringe')
print(path,'path')


fringe_points = []
for points in fringe:
    fringe_points.append(points[0])


x = [point[0] for point in path] 
y = [point[1] for point in path] 

x_e = [pointe[0] for pointe in explored]
y_e = [pointe[1] for pointe in explored]

x_f = [pointf[0] for pointf in fringe_points]
y_f = [pointf[1] for pointf in fringe_points]


x_e = np.array(x_e)
y_e = np.array(y_e)


# Create figure and axis
fig, ax = plt.subplots()

ax.plot(x_e, y_e, marker='.', linestyle='none', color='y')
ax.plot(x_f, y_f, marker='.', linestyle='none', color='b')
ax.plot(x, y, marker='.', linestyle='-.', color='r')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_aspect('equal')

cmap = {100: 'black', 0: 'white', -1: 'lightblue'}
cmap_object = plt.matplotlib.colors.ListedColormap([cmap[val] for val in sorted(cmap.keys())])
bounds = sorted(cmap.keys()) + [max(cmap.keys()) + 1]
norm = plt.matplotlib.colors.BoundaryNorm(bounds, len(bounds) - 1)
plt.imshow(map_grid.map.map_grid, cmap=cmap_object, norm=norm, interpolation='nearest')
plt.colorbar(ticks=sorted(cmap.keys()))
plt.gca().invert_yaxis()

plt.show()
