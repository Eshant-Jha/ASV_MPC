# -*- coding: utf-8 -*-
"""
Created on Thu Mar 14 05:22:15 2024

@author: ESHANT
"""
import numpy as np
import search 
#import successor 
import nextstep
#import successor_genration
#import bezier_curve_generate
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt 


#scenario =successor.Arena(1, 0)
scenario =nextstep.Arena(1,0)
#give non diementionalised starting point 
#starting =np.array([0.001,0,0,0,0.0015,0.001,0.001])
starting= np.array([1,1,0.001])
#aStarSearch(scenario, robot_id, start_state, planner, goal_states):

reference_path = [[0,0], [200,0], [200,200]]
path_data= search.aStarSearch(scenario,1,starting,1,[])

print(path_data)

x_coords = [point[0] for point in path_data]
y_coords = [point[1] for point in path_data]

x_ref  =  [point[0] for point in reference_path]
y_ref =   [point[1] for point in reference_path]

# Plot the path
plt.figure(figsize=(10, 6))
plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b')
plt.plot(x_ref, y_ref, marker='o', linestyle='-', color='r')

# Adding labels and title
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Path Plot')
plt.grid(True)
plt.show()