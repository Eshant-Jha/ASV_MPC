# -*- coding: utf-8 -*-
"""
Created on Tue Feb 27 09:19:57 2024

@author: ESHANT
"""
from scipy.integrate import solve_ivp
import matplotlib
import matplotlib.pyplot as plt
import copy
import numpy as np
import pandas as pd


import matsya_mmg

    
class matsya_mmg_wrap():
    
    
     def __init__(self,
                wind_flag=0,
                wind_speed=0,
                wind_dir=0,
                wave_flag=0,
                wave_height=0,
                wave_period=0,
                wave_dir=0,
                obs_state=None):
        
       
        self.wind_flag = wind_flag
        self.wind_speed = wind_speed
        self.wind_dir = wind_dir

        self.wave_flag = wave_flag
        self.wave_height = wave_height
        self.wave_period = wave_period
        self.wave_dir = wave_dir
        
        self.obs_state = obs_state 
        
                     # Dimensional state
        self.sim_state = obs_state                        # Non dimensional state
        
        self.scale = 75.5
        self.L = matsya_mmg.L/self.scale
        self.U = matsya_mmg.Fn * np.sqrt(matsya_mmg.g * self.L)

        self.rate_value = 100
        
        self.time_step = 0.5
        
        
        self.forward_primitive = []
        #self.time_step = 0.00466  ################how you reached this value ?

        self.delta_c = 0        
        self.sim_state=np.array([1e-6, 0, 0, 0, 0, 0 , 0])  #-np.pi/2
        
     def ssa(ang, deg=False):
        
        if deg:
            ang = (ang + 180) % (360.0) - 180.0
        else:
            ang = (ang + np.pi) % (2 * np.pi) - np.pi
        return ang
    
    
     def step(self,action):
        
        tspan = (0, self.time_step)
        yinit = self.sim_state
        #using i deciding 
        if action==0:
           self.delta_c = 0 
           
        if action==-1:
         self.delta_c = -(10.0 / 180.0) * np.pi
         
         
        if action==1:
            self.delta_c = (10.0 / 180.0) * np.pi
            
            
        print("Rudder angle applied now",self.delta_c*(180/np.pi))
        '''Motion_primitive_Forward'''
        
        #### Delta value same as initial value
        
        sol = solve_ivp(lambda t,v: matsya_mmg.matsya_ode(t,v,self.delta_c, wind_flag=self.wind_flag,
                                                wind_speed=self.wind_speed, wind_dir=self.wind_dir,
                                                wave_flag=self.wave_flag, wave_height=self.wave_height,
                                                wave_period=self.wave_period, wave_dir=self.wave_dir),
                        tspan, yinit, dense_output=True)
        
        u = sol.y[0, -1]*self.U
        v = sol.y[1, -1]*self.U
        r = sol.y[2, -1]*self.U /self.L
        x = sol.y[3, -1]*self.L
        y = sol.y[4, -1]*self.L
        psi_rad = sol.y[5, -1]  # psi
        
        psi = (psi_rad + np.pi) % (2 * np.pi) - np.pi
        
        delta = sol.y[6, -1]
        
        self.new_primitive_f = [x,y,  np.rad2deg(psi) , delta*(180/np.pi)]
        
        self.forward_primitive.append(self.new_primitive_f)
        self.sim_state = np.array([sol.y[0, -1], sol.y[1, -1], sol.y[2, -1], sol.y[3, -1], sol.y[4, -1], psi, sol.y[6, -1]])
        obs_state = np.array([u, v, r, x, y, psi, delta])    # This state is in NED
        
    
        self.obs_state = obs_state
        
        return self.new_primitive_f
      
if __name__=="__main__":
    
    f,l,r=[],[],[]
    matsya=matsya_mmg_wrap()
    
    for i in range(10) :
        f.append(matsya.step(0))
       
        
    matsya2=matsya_mmg_wrap()
    
    for i in range(10):
        l.append(matsya2.step(-1))
        
    matsya3=matsya_mmg_wrap()

    for i in range(10):
         r.append(matsya3.step(1))

        
# Printing the lists
# =============================================================================
#     print("Results for Forward Action (0):")
#     for i, val in enumerate(f):
#         print(f"Step {i+1}: {val}")
#     
# =============================================================================


    print("\n \nResults for Left Action (-1):")
    for i, val in enumerate(l):
        print(f"\nStep {i+1}: {val}")
    
    print("\n \nResults for Right Action (1):")
    for i, val in enumerate(r):
        print(f"\nStep {i+1}: {val}")
    
    f_x_positions = [item[0] for item in f]
    f_y_positions = [item[1] for item in f]
    
    l_x_positions = [item[0] for item in l]
    l_y_positions = [item[1] for item in l]
    
    r_x_positions = [item[0] for item in r]
    r_y_positions = [item[1] for item in r]
 
    plt.figure(figsize=(8, 6))
 
    # Plotting all actions
    plt.plot(f_x_positions, f_y_positions, marker='o', label='Forward (0)', color='blue')
    plt.plot(l_x_positions, l_y_positions, marker='o', label='Left (-1)', color='green')
    plt.plot(r_x_positions, r_y_positions, marker='o', label='Right (1)', color='red')
    
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title(f'Ship\'s Position Over Time for Different Actions (Time Step: {matsya.time_step} second)')
    plt.grid(True)
    plt.legend()
   
    plt.show()
    
   # Create a DataFrame for each action
   
    df_f = pd.DataFrame(f, columns=['X', 'Y', 'Heading (deg)', 'Rudder angle (deg)'])
    df_l = pd.DataFrame(l, columns=['X', 'Y', 'Heading (deg)', 'Rudder angle (deg)'])
    df_r = pd.DataFrame(r, columns=['X', 'Y', 'Heading (deg)', 'Rudder angle (deg)'])
    
    # Export DataFrames to Excel
    
    with pd.ExcelWriter('ship_data.xlsx', engine='xlsxwriter') as writer:
        
        df_f.to_excel(writer, sheet_name='Forward Action', index=False)
        df_l.to_excel(writer, sheet_name='Left Action',    index=False)
        df_r.to_excel(writer, sheet_name='Right Action',   index=False)
    
    
      
# class Maze:
#   """
#   This class outlines the structure of the maze problem
#   """
  
#   maze_map = []# To store map data, start and goal points
  
#   # [delta_x, delta_y, description]
#   forward=[]
#   five_neighbor_actions = {'up':[-1, 0],'left': [0, -1], 'right': [0, 1] } #'stop': [0, 0]}
    
 
      
#   # default constructor
#   def __init__(self, id):
#       """
#       Sets the map as defined in file maze_maps
#       """
#       #Set up the map to be used
#       self.maze_map = maze_map.maps_dictionary[id]  
      
#       return
     
#   def getStartState(self, robot_id):
#      """
#      Returns the start state for the search problem 
#      """
#      if robot_id==1:
#       start_state = self.maze_map.r1_start 
#      elif robot_id==2:
#       start_state= self.maze_map.r2_start
#      else:
#        start_state= self.maze_map.r3_start  
#      return start_state
 
#   def getGoalState(self,robot_id):
#      """
#      Returns the start state for the search problem 
#      """
#      if robot_id==1:
#          goal_state = self.maze_map.r1_goal
#      elif robot_id==2:
#          goal_state= self.maze_map.r2_goal
#      else:
#          goal_state =self.maze_map.r3_goal
         
#      return goal_state
    
#   def isGoalState(self, robot_id, state):
#      """
#        state: Search state
    
#      Returns True if and only if the state is a valid goal state
#      """
#      if state[0:2] == self.getGoalState(robot_id):
#          return True
#      else:
#          return False

#   def isObstacle(self, state):
#       """
#         state: Search state
     
#       Returns True if and only if the state is an obstacle
     
#       """

#       if self.maze_map.map_data[state[0]][state[1]] == maze_map.obstacle_id:
#           return True
#       else:
#           return False
    
  

#    #Check the p3 point position w.r.t to line joining p1 to p2
#   def point_position_with_line(self,p1, p2, p3):
#       # p1 is the collision point
#       # p2 is a point on the line joining collision and other bot coordinates (P1P2)
#       # p3 is the state in get_successor

#       x1, y1 = p1
#       x2, y2 = p2
#       x3, y3 = p3

#       # Calculate vectors representing the line (P1P2) and the point (P1P3)
#       line_vector = (x2 - x1, y2 - y1)
#       point_vector = (x3 - x1, y3 - y1)

#       # Calculate the cross product of the two vectors
#       cross_product = line_vector[0] * point_vector[1] - line_vector[1] * point_vector[0]
#       return cross_product>=0   #will return only when point is on left
  
    
#   def getSuccessors(self, state): #this is used in global planning 
 
#      """
#        state: Seacrh state
     
#      For a given state, this should return a list of triples, 
#      (successor, action, stepCost), where 'successor' is a 
#      successor to the current state, 'action' is the action
#      required to get there, and 'stepCost' is the incremental 
#      cost of expanding to that successor
#      """
#      successors = []  
#      for action in self.five_neighbor_actions:
         
#          #Get individual action
#          del_x, del_y = self.five_neighbor_actions.get(action) 
        
#          #Get successor
#          new_successor = [state[0] + del_x , state[1] + del_y]   
#          new_action = action
         
#          # Check for static obstacle 
         
#          if self.isObstacle(new_successor):
#              continue
      
#          #cost
#          new_cost = maze_map.free_space_cost         
#          successors.append([new_successor, new_action, new_cost])
         
#      return successors
 
    
#   def getSuccessors_local(self, robot_id, state, collison_point, neighbor_robot_point, start_state): #used for local planning 
#      """
#        state: Seacrh state
      
#     to be added in this code FOR HEAD on COLLISION BOTH REPLANS CONSIDERING VECTOR FROM CURRENT TO COLLISON AND TAKING LEFT SIDE SUCCESSOR INVALID
#      """
#      #print("local planning doing for ",robot_id)
#      successors = []  
#      for action in self.five_neighbor_actions:
         
#          #Get individual action
#          del_x, del_y = self.five_neighbor_actions.get(action) 
            
#          #Get successor
#          new_successor = [state[0] + del_x , state[1] + del_y]    #new_successor = [state[0] + del_x , state[1] + del_y, state[2]+1]
#          new_action = action
         
#          #Check for static obstacle 
#          if self.isObstacle(new_successor):
#              continue
         
#          #COLREGS rule 15 applied for collision avoidance with other bot /here its making obstacle zone 

#          if  self.point_position_with_line(start_state,neighbor_robot_point, new_successor) and \
#                self.point_position_with_line(neighbor_robot_point,collison_point ,new_successor):

#                continue
            
#          new_cost = maze_map.free_space_cost  
#          #print(new_successor)            
#          successors.append([new_successor, new_action, new_cost])
         
#      return successors
    
      
#   def getSuccessors_head_on(self, robot_id, state, collison_point, neighbor_robot_point, start_state):    
      

#      #HERE WE HAVE TO ENSURE  START STATE AND NEIGHBOUR STATE RESPECTIVE POINTS JUST BEFORE COLLISION TO HAVE 180  
#      successors = []  
#      for action in self.five_neighbor_actions:
         
#          #Get individual action
#          del_x, del_y = self.five_neighbor_actions.get(action) 
            
#          #Get successor
#          new_successor = [state[0] + del_x , state[1] + del_y]    #new_successor = [state[0] + del_x , state[1] + del_y, state[2]+1]
#          new_action = action
         
#          #Check for static obstacle 
#          if self.isObstacle(new_successor):
#              continue
         
#          #COLREGS rule 15 applied for collision avoidance with other bot /here its making obstacle zone 
         
#          point_a =np.array(start_state)

#          point_b = np.array(neighbor_robot_point)
         
#          test_point= np.array(new_successor)
#         # Vector from A to B
#          ab_vector = point_b - point_a

#         # Vector from A to test point
#          ap_vector = test_point - point_a

#          # Vector from B to test point

#          # Calculate cross products
#          cross_product_ap = np.cross(ab_vector, ap_vector)
        
#         # Check conditions
#          if cross_product_ap >=0:
#             if np.linalg.norm(ap_vector)<=2.3:

#               #print("value headon ")      
#               continue
         
#          new_cost = maze_map.free_space_cost  
#          #print(new_successor)            
#          successors.append([new_successor, new_action, new_cost])
         
#      return successors 
      
      

    
      
      
      
      
      
      