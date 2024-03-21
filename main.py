# -*- coding: utf-8 -*-
"""
Created on Thu Mar 14 05:22:15 2024

@author: ESHANT
"""

import numpy as np
import search 
import successor 
import successor_genration
import bezier_curve_generate
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt 


problem=successor.Maze(1, 0)
#path= search.aStarSearch(problem, robot_id, start_state, planner, collison_point, neighbor_robot_point, goal_states)
path= search.aStarSearch(problem, 1, [1,1,0], 1, [], [], [])
print(path)

# Extract x, y, and orientation (in the 2D plane)
x = [point[0] for point in path]
y = [point[1] for point in path]
orientation = [point[2] for point in path]



# Define obstacle parameters
obstacle_x = 3
obstacle_y = 3
obstacle_radius = 0.2

# Create obstacle circle
theta = np.linspace(0, 2*np.pi, 100)
obstacle_circle_x = obstacle_x + obstacle_radius * np.cos(theta)
obstacle_circle_y = obstacle_y + obstacle_radius * np.sin(theta)


# Create figure and axis
fig, ax = plt.subplots()

# Plot points and connect with lines
ax.plot(x, y, marker='x', linestyle='--', color='r')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.plot(obstacle_circle_x, obstacle_circle_y, 'g--', label='Obstacle (20, 35, r=15)')
plt.scatter(obstacle_x, obstacle_y, c='g', label='Obstacle Center (20, 35)')

# Show plot
plt.show()
#rudder angle to be re-initialised or kept zero after it reaches local waypoint with the heading provided
 #Extract x and y coordinates
# =============================================================================
#  
# # Extract x and y coordinates from the path
# x = [point[0] for point in path]
# y = [point[1] for point in path]
# 
# # Create a smooth curve using interpolation
# x_smooth = np.linspace(min(x), max(x), 1000)
# f_smooth = interp1d(x, y, kind='cubic')
# y_smooth = f_smooth(x_smooth)
# 
# # Define obstacle parameters
# obstacle_x = 20
# obstacle_y = 35
# obstacle_radius = 15
# 
# # Create obstacle circle
# theta = np.linspace(0, 2*np.pi, 100)
# obstacle_circle_x = obstacle_x + obstacle_radius * np.cos(theta)
# obstacle_circle_y = obstacle_y + obstacle_radius * np.sin(theta)
# 
# # Plot the smooth curve with obstacle
# plt.figure(figsize=(10, 6))
# plt.plot(x_smooth, y_smooth, 'r-', label='Smooth Curve')
# plt.scatter(x, y, c='b', s=10, label='Path Points')  # Plot the original points as well
# plt.plot(obstacle_circle_x, obstacle_circle_y, 'g--', label='Obstacle (20, 35, r=15)')
# plt.scatter(obstacle_x, obstacle_y, c='g', label='Obstacle Center (20, 35)')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Path Plot with Obstacle')
# plt.legend()
# plt.grid(True)
# plt.show() 
# =============================================================================
 
# =============================================================================
#  
#  
# x = [point[0] for point in path]
# y = [point[1] for point in path]
# 
# # Create a smooth curve using interpolation
# x_smooth = np.linspace(min(x), max(x), 1000)
# f_smooth = interp1d(x, y, kind='cubic')
# y_smooth = f_smooth(x_smooth)
# 
# 
# 
# # Plot the smooth curve
# plt.figure(figsize=(10, 6))
# plt.plot(x_smooth, y_smooth, 'r-')
# plt.scatter(x, y, c='b', s=10)  # Plot the original points as well
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Path Plot (Smooth Curve)')
# plt.grid(True)
# plt.show()
# =============================================================================
 
#move to goal 

def gtg(self, robot_state, goal_state):  
    
    
        #The Go to goal controller
        
        #determine how far to rotate to face the goal point
        
        
        #PS. ALL ANGLES ARE IN RADIANS
        delta_theta = (np.arctan2((goal_state[1] - robot_state[1]), (goal_state[0] - robot_state[0]))) - robot_state[2]
        
        
        #restrict angle to (-pi,pi)
        delta_theta = ((delta_theta + np.pi)%(2.0*np.pi)) - np.pi
        
        
        #Error is delta_theta in degrees
        e_new = ((delta_theta*180.0)/np.pi)
        e_dot = (e_new - self.prev_heading_error)/self.dt 
        self.total_heading_error = (self.total_heading_error + e_new)*self.dt
                    
      
        #rudder angle only 
        #find distance to goal
        d = np.sqrt(((goal_state[0] - robot_state[0])**2) + ((goal_state[1] - robot_state[1])**2))
        
        #velocity parameters
        distThresh = 0.1#mm
        
        
        #request robot to execute velocity
        
        return commanded_rudder_angle 
    

