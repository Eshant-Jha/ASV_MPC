# -*- coding: utf-8 -*-
"""
Created on Fri Mar  8 11:55:52 2024

@author: ESHANT
"""
import matsya_mmg
from  bezier_curve_generate import Bezier
from  successor_genration   import matsya_mmg_wrap
import numpy as np
  
#k=Bezier(id, 30).curve_points(1,30)
#print(k)

#print("bhai mein aagya ",curve)

class Maze:

    
   """
  This class outlines the structure of the maze problem
   """
   maze_map = []# To store map data, start and goal points
   
   # [delta_x, delta_y, description] 
   
   #curve = bezier_curve_generate.bezier_curve_left
   #getting simulated points here 
   
   
  
   
   def __init__(self, id ,heading_angle):
       
       self.f,self.l,self.r=[],[],[]

       matsya=matsya_mmg_wrap()
       self.id =id
       for i in range(10) :
           self.f.append(matsya.step(0))
          
           
       matsya2=matsya_mmg_wrap()
       
       for i in range(10):
           self.l.append(matsya2.step(1))
           
           
       matsya3=matsya_mmg_wrap()

       for i in range(10):
            self.r.append(matsya3.step(-1))
       
       
       #print(fx,fy,lx,ly,"succesor")
   
   def getSuccessors(self, state):
        
        x,y ,psi = state 
        successors=[]
        self.theta =np.deg2rad(state[2])
        
        #print("heading angle is now  ",state[2])
        self.R = np.array([
             [np.cos(self.theta), -np.sin(self.theta)],
             [np.sin(self.theta), np.cos(self.theta)]
         ])
        fx,fy,fpsi = [self.f[-1][0],self.f[-1][1],self.f[-1][2]]        
        lx,ly,lpsi = [self.l[-1][0],self.l[-1][1],self.l[-1][2]]  
        rx,ry,rpsi = [self.r[-1][0],self.r[-1][1],self.r[-1][2]]  
        # as per the heading angle fx , fy components will change , ignoring affect on fpsi 
        print()
        delta_action= np.array([[fx, fy],[lx,ly],[rx,ry]])
        delta_action2= np.array([[fx, fy,fpsi],[lx,ly,lpsi],[rx,ry,rpsi]])
        print(delta_action2, "action initial" )
        rotated_action =np.dot(delta_action, self.R.T)
        
        fx ,fy =  rotated_action[0]
        lx , ly = rotated_action[1]
        rx , ry = rotated_action[2]
        #considering change in heading angle is independent of the instant heading  of ship  
        self.three_actions = {'forward':[fx,fy,fpsi],'left': [lx, ly,lpsi], 'right': [rx, ry,rpsi] }
        print()
        print()
        print(self.three_actions,"after rotation as per heading ")    
          
        
        for action in self.three_actions:
            
            del_x ,del_y , del_psi = self.three_actions.get(action)
            
            ang = psi + del_psi
            
            ang = (ang + 180) % (360.0) - 180.0
            #print(del_x,del_y,"dekho ")
            new_successor = [x + del_x , y + del_y, ang ]
            
            #new_successor2 =[x + del_x/2 , y + del_y/2, psi + del_psi]
            
            
            new_action = action
            
            
    
                    
                   
            if self.isObstacle(new_successor,action):
                
               print(action)
               continue
# =============================================================================
#             if self.isObstacle(new_successor2, action):
#                 continue 
# =============================================================================
            #cost
            
            new_cost = 8
            #print(action)
            successors.append([new_successor, new_action, new_cost])
            #print(successors)
                    
        return successors 
            
       
   def isObstacle(self,state,action):
       
        
      self.threshold_distance = 1
      # having one obstaclea at 5 , 5 
      x , y = 3 , 3 
      
      '''Using x, y coordinates we would check whether if it lies in obstacle region
      ,obstacle region points would be stored previously ''' 
      
      distance = np.sqrt((x - state[0])**2 + (y - state[1])**2)
      
      if distance <= self.threshold_distance:
          print(state , "its obstacle")
          return True 
      else:
          return  False 
          
          
# =============================================================================
#       if self.maze_map.map_data[state[0]][state[1]] == maze_map.obstacle_id:
#            return True
#       else:
#           return False
# =============================================================================
      

   def distance_to_bezier(self,point, bezier_curve):
        
        # Compute distance between point and each point on the curve
        distances = [distance.euclidean(point, p) for p in bezier_curve]
        min_distance = min(distances)
        if min_distance:
            
          return min_distance 
   def getGoalState(self,robot_id):
         """
         Returns the start state for the search problem 
         """
         if robot_id==1:
             goal_state = [10,0,0]#self.maze_map.r1_goal
         elif robot_id==2:
             
             goal_state= self.maze_map.r2_goal
             
         else:
             goal_state =self.maze_map.r3_goal
            
         return goal_state
   
        
   def isGoalState(self, robot_id, state):
   #      """
   #        state: Search state
       
   #      Returns True if and only if the state is a valid goal state
   #      """
         if abs(state[0] - 10) + abs(state[1] - 0) < 1 : #self.getGoalState(robot_id):
             return True
         else:
             return False    

