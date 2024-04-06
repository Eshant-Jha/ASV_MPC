# -*- coding: utf-8 -*-
"""
Created on Fri Mar  8 11:55:52 2024

@author: ESHANT
@efficiently edited by: THE GREAT DON
"""

import numpy as np
from nav_msgs.msg import  OccupancyGrid
import map_grid
  


class Problem:

    
    """
    This class outlines the structure of the maze problem
    """
    map_grid = []




    def __init__(self):
       
        self.f,self.ten_l,self.ten_r,self.twenty_r,self.twenty_l=[],[],[],[],[]





        '''         used as old motion primitives

        self.f = [[0.06299997541643738, 0.0, 0.0,0.0 ], 
                  [0.9633015258572852, 0.0, 0.0, 0.0], 
                   [2.0929804835999977, 0.0, 0.0, 0.0],
                  [2.796194699489363, 0.0, 0.0, 0.0], 
                  [3.582415346988044, 0.0, 0.0, 0.0], 
                  [4.444861124356223, 0.0, 0.0, 0.0], 
                  [5.376976250287516, 0.0, 0.0, 0.0]]     
        self.l = [[0.061878094603640396, -0.0004510590605578181, 1.0888551465850074, 10.102597535402003],  
                  [0.9365673308789444, 0.06814344795093481, 8.938502845198498, 10.102079520101976], 
                  [2.0141573077452772, 0.27237415119112485, 15.354331446901735, 10.091279579633534], 
                  [2.6709866686202433, 0.45175792932772263, 18.909171964230694, 10.019885017230264], 
                  [3.390290724502584, 0.6950573152769727, 22.7066603812417, 9.864865400104012], 
                  [4.1585084189149155, 1.0106934305039978, 26.75087836617968, 9.965555388777862], 
                  [4.961045347569757, 1.4060404968137252, 31.043072120095673, 10.152488422258795]]
        self.r = [[0.06183979841330793, 0.00048398252998552296, -1.1204406548301962, -10.129009406490415],  
                  [0.9341403431575229, -0.07173062680280669, -9.56426089970077, -10.086171212301364], 
                  [2.0047399483182207, -0.2900714545582973, -16.61646953118392, -9.977313444474087], 
                  [2.6543593762232724, -0.4828878824776647, -20.561529054446016, -10.142700532757704], 
                  [3.3623835451380333, -0.7450933375499272, -24.802460965024657, -10.02930533090411], 
                  [4.113626655723127, -1.0858474001943403, -29.346170949010983, -9.880431918674308], 
                  [4.891513216350091, -1.5129917856539166, -34.19494213178685, -9.985379313082818]]     '''
        
        self.f = [[0.06299997541643738, 0.0, 0.0,0.0 ], 
                  [12.1847322, 0.0, 0.0, 0.0]]
        self.ten_l = [[0.06299997541643738, 0.0, 0.0,0.0 ], 
                  [11.4899337,	2.186791127,	38.38626691]]
        self.ten_r = [[0.06299997541643738, 0.0, 0.0,0.0 ], 
                  [11.31995916	,-2.386289743,	-43.51630406]]
        
        self.twenty_l = [[0.06299997541643738, 0.0, 0.0,0.0 ], 
                  [10.26303055	,3.422383216,	63.73175065,	19.93423815]]
        self.twenty_r = [[0.06299997541643738, 0.0, 0.0,0.0 ], 
                  [10.02278982,	-3.444872016,	-66.26462692,	-20.2255009]]



        self.map_grid  = map_grid.maps_dictionary[1] 

        return







    def getSuccessors(self, state):
        

        x,y ,psi = state 
        successors=[]
        self.theta =np.deg2rad(psi)
        
        #print("heading angle is now  ",state[2])
        self.R = np.array([
             [np.cos(self.theta), -np.sin(self.theta)],
             [np.sin(self.theta), np.cos(self.theta)]
         ])
        fx,fy,fpsi = [self.f[-1][0],self.f[-1][1],self.f[-1][2]]        
        ten_lx,ten_ly,ten_lpsi = [self.ten_l[-1][0],self.ten_l[-1][1],self.ten_l[-1][2]]  
        ten_rx,ten_ry,ten_rpsi = [self.ten_r[-1][0],self.ten_r[-1][1],self.ten_r[-1][2]]  

        twenty_lx,twenty_ly,twenty_lpsi = [self.twenty_l[-1][0],self.twenty_l[-1][1],self.twenty_l[-1][2]]  
        twenty_rx,twenty_ry,twenty_rpsi = [self.twenty_r[-1][0],self.twenty_r[-1][1],self.twenty_r[-1][2]]  
        # as per the heading angle fx , fy components will change , ignoring affect on fpsi 
        # print()
        delta_action= np.array([[fx, fy], [ten_lx,ten_ly],[ten_rx,ten_ry] ,  [twenty_lx,twenty_ly],[twenty_rx,twenty_ry] ])


        rotated_action =np.dot(delta_action, self.R.T)
        
        fx ,fy =  rotated_action[0]
        ten_lx , ten_ly = rotated_action[1]
        ten_rx , ten_ry = rotated_action[2]
        twenty_lx , twenty_ly = rotated_action[1]
        twenty_rx , twenty_ry = rotated_action[2]


        #considering change in heading angle is independent of the instant heading  of ship  
        self.five_actions = {'forward':[fx,fy,fpsi],'ten_left': [ten_lx, ten_ly,ten_lpsi], 'ten_right': [ten_rx, ten_ry,ten_rpsi] 
                            ,'twenty_left': [twenty_lx, twenty_ly,twenty_lpsi], 'twenty_right': [twenty_rx, twenty_ry,twenty_rpsi] }

        
        for action in self.five_actions:       # action in self.three_actions:
            
            del_x ,del_y , del_psi = self.five_actions.get(action)
            
            ang = psi + del_psi
            
            ang = (ang + 180) % (360.0) - 180.0

            new_successor = [x + del_x , y + del_y, ang ]

            new_action = action
            

            def between_points(x1, y1, x2, y2, num_points):
                dx, dy = (x2 - x1) / (num_points + 1), (y2 - y1) / (num_points + 1)
                return [[x1 + i * dx, y1 + i * dy] for i in range(1, num_points + 1)]


            x1, y1 = state[0], state[1]
            x2, y2 = new_successor[0],new_successor[1]
            num_points = 10
            interpolated_points = between_points(x1, y1, x2, y2, num_points)
    
                    
                   
            if any(self.isObstacle(point,action) for point in interpolated_points):



















                continue


            new_cost = 1
            # print(action)
            # print(new_successor,'neusuccessor')
            successors.append([new_successor, new_action, new_cost])
            # print(successors)
                    
        return successors 
            
       

    def isGoalState(self, robot_id, state):
    #      """
    #        state: Search state
        
    #      Returns True if and only if the state is a valid goal state
    #      """
        diff =[state[0] - self.getGoalState(robot_id)[0] , state[1] - self.getGoalState(robot_id)[1]]  
        eucl = ( diff[0]**2 +diff[1]**2  )**0.5
        psi_diff = abs(state[2] - self.getGoalState(robot_id)[2])
        if eucl < 3   :#  and psi_diff < 18:    #self.getGoalState(robot_id):   this is because +- half of the one step neede to given as tolerence
             
             return True
        else:
             return False    
        

    def getGoalState(self,robot_id):
         """
         Returns the start state for the search problem 
         """
         if robot_id==1:
             goal_state = self.map_grid.goal
             
         return goal_state
   
    def getStartState(self, robot_id):
        """
        Returns the start state for the search problem 
        """
        if robot_id==1:
            start_state = self.map_grid.start

        return start_state   


 

    def isObstacle(self, state,action):
        """
            state: Search state
        
        Returns True if and only if the state is an obstacle
        
        """
        x = state[0]
        x = round(state[0])
        y = state[1]
        y = round(state[1])

        if self.map_grid.map_grid[y][x] == map_grid.obstacle_id:
            # print(state[0],state[1],'obstacle')
            return True
        else:
            return False    
        

       
        


        


