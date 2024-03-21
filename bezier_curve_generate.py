# -*- coding: utf-8 -*-
"""
Created on Thu Mar  7 18:24:12 2024

@author: ESHANT
"""
#to be used as succesor generation 
#This curve is generated for the ship simulated using mmg for time step= 0.5 it is matched to curve for 5 sec duration 
#Right side turn using 10 degree rudder angle  
#n+1 control point gives n order curve 

import numpy as np
import matplotlib.pyplot as plt


class Bezier:
    
    
    def __init__(self,id,angle):
    
        # [0.93, 0.0], Define control points for forward curve
        
        self.P_right = np.array([
            [0, 0],
           
            [3.01, 0.27],
            [4.96, 1.40]
        ])
        
        self.theta = np.radians(angle) #heading angle as per global frame
        
        # Define the rotation matrix
        self.R = np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]
        ])
        
        # [0.93, 0.0], Define control points for left curve
        self.P_left = np.array(
            [[0  ,  0   ], 
            [3.01, -0.27],
            [4.96, -1.40]
        ])
        
        #  [1, 0.0],Define control points for right curve
        self.P_forward = np.array([[0, 0],[3, 0],[5.39, 0]])
    
    # Apply the rotation as heading angle in global frame 
    
        self.P_right   = np.dot(self.P_right, self.R.T)
        self.P_left    = np.dot(self.P_left,self.R.T)
        self.P_forward = np.dot(self.P_forward, self.R.T)
    
    def bernstein_matrix(self, n, t):
        # Initialize matrix
        B = np.zeros((n+1, len(t)))
        
        # Calculate Bernstein basis functions
        for i in range(n+1):
            B[i] = np.math.comb(n, i) * t**i * (1 - t)**(n-i)
        
        return B
    
    def curve_points(self,action ,angle):
        
        
        # Define parameter t (step size = 1/100)
        t = np.linspace(0, 1, 100)
        curve= Bezier(id, 30)
        # Compute Bernstein matrix
        B = curve.bernstein_matrix(2, t)
        
        # Construct Bézier curve for forward
        self.bezier_curve_forward = np.dot(B.T, curve.P_forward)
        #print(bezier_curve_forward)

        # Construct Bézier curve for left
        self.bezier_curve_left = np.dot(B.T,curve.P_left)

        # Construct Bézier curve for right
        self.bezier_curve_right = np.dot(B.T, curve.P_right)
        if action==1:
            return self.bezier_curve_left
        
        
# =============================================================================
# # Define parameter t (step size = 1/100)
# t = np.linspace(0, 1, 100)
# curve= Bezier(id, 30)
# # Compute Bernstein matrix
# B = curve.bernstein_matrix(2, t)
# 
# # Construct Bézier curve for forward
# bezier_curve_forward = np.dot(B.T, curve.P_forward)
# print(bezier_curve_forward)
# 
# # Construct Bézier curve for left
# bezier_curve_left = np.dot(B.T,curve.P_left)
# 
# # Construct Bézier curve for right
# bezier_curve_right = np.dot(B.T, curve.P_right)
# #print(bezier_curve_right)
# =============================================================================
#obstacle
# =============================================================================
# # Plot the curves with control points
# plt.plot(bezier_curve_forward[:, 0], bezier_curve_forward[:, 1], label='Forward Bézier Curve')
# plt.plot(bezier_curve_left[:, 0], bezier_curve_left[:, 1], label='Left Bézier Curve')
# plt.plot(bezier_curve_right[:, 0], bezier_curve_right[:, 1], label='Right Bézier Curve')
# plt.scatter(curve.P_forward[:, 0], curve.P_forward[:, 1], color='blue', label='Forward Control Points', zorder=5)
# plt.scatter(curve.P_left[:, 0], curve.P_left[:, 1], color='green', label='Left Control Points', zorder=5)
# plt.scatter(curve.P_right[:, 0], curve.P_right[:, 1], color='red', label='Right Control Points', zorder=5)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Bézier Curves for Forward, Left, and Right')
# plt.legend()
# plt.grid(True)
# plt.show()
# 
# =============================================================================
