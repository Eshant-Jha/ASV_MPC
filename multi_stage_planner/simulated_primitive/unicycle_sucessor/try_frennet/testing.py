# -*- coding: utf-8 -*-

"""
Created on Fri May 31 10:50:39 2024

@author: ESHANT
"""

import numpy as np

class Cross_track:
    
    def point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        # Vector AB
        A = np.array([x1, y1])
        B = np.array([x2, y2])
        P = np.array([px, py])
        
        AB = B - A
        AP = P - A
        BP = P - B
        
        # Projection of AP onto AB
        AB_norm = np.dot(AB, AB)
        if AB_norm == 0:
            return np.linalg.norm(AP)  # A == B case
        
        projection = np.dot(AP, AB) / AB_norm
        
        if projection < 0:
            # Closest to A
            return np.linalg.norm(AP)
        elif projection > 1:
            # Closest to B
            return np.linalg.norm(BP)
        else:
            # Closest to line segment
            closest_point = A + projection * AB

            return np.linalg.norm(P - closest_point)
    
    def cross_track_error(self, path, point):
        x0, y0 = point
        min_distance = float('inf')
         
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            
            distance = self.point_to_segment_distance(x0, y0, x1, y1, x2, y2)
            if distance < min_distance:
                min_distance = distance
        
        return min_distance
    

    def guidance(self):
        
        # Waypoint switching
        if np.linalg.norm(self.x_hat[6:9] - self.goal_waypoint / self.length) < 3 and self.terminate_flag is False:
            
            self.waypoint_index  += 1

            if self.waypoint_index == self.waypoints.shape[0] - 1:
                self.terminate_flag = True

            else:  

                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.goal_waypoint = self.waypoints[self.waypoint_index + 1]

        # Guidance mechanism
        pi_p = np.arctan2(self.goal_waypoint[1] - self.current_waypoint[1], 
                        self.goal_waypoint[0] - self.current_waypoint[0])

        R_n_p = np.array([[np.cos(pi_p), -np.sin(pi_p)], [np.sin(pi_p), np.cos(pi_p)]])
        
        x_n_i = self.current_waypoint[0:2] / self.length
        x_n = self.x_hat[6:8]

        xy_p_e = R_n_p.T @ (x_n - x_n_i)
        self.x_p_e = xy_p_e[0]
        self.y_p_e = xy_p_e[1]



