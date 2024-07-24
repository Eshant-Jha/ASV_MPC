# -*- coding: utf-8 -*-

"""
Created on Thu Mar 14 05:22:15 2024

@author: ESHANT
"""

import numpy as np
import search 
from guidance_control  import Agent
import nextstep
import matplotlib.pyplot as plt 
from scipy.interpolate import CubicSpline 

reference_path = [[0,0], [200,0], [200,200]]

scenario =nextstep.Arena(1,0)

starting= np.array([1,1,0.001])
#aStarSearch(scenario, robot_id, start_state, planner, goal_states):
path_data= search.aStarSearch(scenario,1,starting,1,[])

path_coord = path_data[0]
path_coord=np.array(path_coord)


# Generate a parameter t for the path
t = np.linspace(0, 1, len(path_coord))

# Create parametric splines  as functions of t
spl_path = CubicSpline(t, path_coord, bc_type="clamped")
#spl_path(t) gives coordinate of point on curve at "t" t E (0,1) 



# Plotting the spline
t_plot = np.linspace(0, 1, 500)
x_plot = spl_path(t_plot)

plt.figure(figsize=(8, 6))
plt.plot(path_coord[:, 0], path_coord[:, 1], 'o', label='Waypoints')
plt.plot(x_plot[:, 0], x_plot[:, 1], label='Cubic Spline Path')
plt.title('Parametric Cubic Spline Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()



# NOW SIMULATION PART BEGINS YOU THE PARAMETRISED PATH NAVIAGTE 
T = 1000
# t = min(x_ex)
time = np.linspace(0,T,10000)
x0,y0,psi0= 1,1,0.11

A = Agent([x0,y0,psi0])

X0  = [1.00,0,0,x0,y0,psi0,0]
xspl= [0,1]    #path_coord[:,0]

Refr1 = A.simulation(spl_path,time,xspl,np.array(X0))
# print(Refr1[0,:])

plt.plot(Refr1[0,:],Refr1[1,:])

plt.show()


x_coords =  [ point[0] for point in path_data[0]]
y_coords =  [ point[1] for point in path_data[0]]

x_ref   =   [ point[0] for point in  reference_path]
y_ref   =   [ point[1] for point in  reference_path]
 
x_xplore=  [ point[0] for point in path_data[1]]
y_xplore = [ point[1] for point in path_data[1]]

# for i in range(len(path_data[2])):
# print(path_data[2][i][0], "fringe is this ")

x_fringe=  [ point[0] for point in path_data[2]]
y_fringe = [ point[1] for point in path_data[2]]
# Plot the path
plt.figure(figsize=(10, 6))

# Plot main coordinates
plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='r', markersize=6, label='Actual')

# Plot reference coordinates
plt.plot(x_ref, y_ref, marker='s', linestyle='-', color='b', markersize=4, label='Reference')

# Plot explored nodes
#plt.plot(x_xplore, y_xplore, marker='*', linestyle='none', color='g', markersize=3, alpha=0.7, label='Explored Nodes')
#plt.plot(x_fringe, y_fringe, marker='*', linestyle='none', color='g', markersize=3, alpha=0.7, label='Explored Nodes')

# Adding legend

plt.legend()

# plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b')
# plt.plot(x_ref, y_ref, marker='o', linestyle='-', color='r')
# #explored nodes being plotted    #need improvement 
# plt.plot(x_xplore, y_xplore, marker='.', linestyle='none', color='o') 


# Adding labels and title
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Path Plot')
plt.grid(True)
plt.show()