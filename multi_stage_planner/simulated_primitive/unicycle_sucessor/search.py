# -*- coding: utf-8 -*-
"""
Created on Thu Mar 14 05:15:29 2024
@author: ESHANT
"""
import numpy as np 
import operator
from  testing import Cross_track
import matplotlib.pyplot as plt 
import time 

reference_path = [[0,0], [200,0], [200,200]]

def heuristic_1(scenario,state,refrence_path):
    
    goal= reference_path[-1]

    error =((goal[1]-state[1])**2 + (goal[0] - state[0]))**(1/2) 
    
    return error
    
def aStarSearch(scenario, robot_id, start_state, planner, goal_states):   
  "Search the node that has the lowest combined cost and weighted heuristic first."
  explored = []
  fringe = []
  #Add the start state to Fringe [start_state ]
  fringe.append([start_state, [start_state[0:2]], 0, (heuristic_1(scenario, start_state, []) ),[]]) #[] added in motion primitive to store action
  print("Planning...")  

  while len(fringe)>0:
      
      fringe = sorted(fringe, key = operator.itemgetter(3))
      
      #Pop least cost node and add to explored list
      current_node = fringe.pop(0)
      explored.append(current_node[0]) # only the state needs to be added to explored list  
      if planner==1:
            
          if scenario.isGoalState(current_node[0],reference_path): 
              #planning only till pre-defined  number of steps 
              #steps = 15   
              #if len(current_node[1])>steps : 
                
                path_coordinates = current_node[1]
                return path_coordinates
          
      if planner==1:  
        
        successors = scenario.getSuccessors(current_node[0])  

        
      for successor, action, cost in successors:
          
          g = current_node[2] + cost
          #g=0

          h = heuristic_1(scenario, successor, reference_path)

          path = current_node[1] + [successor[0:2]]
          temp_node = [successor, path, g, h+g] 
          temp_node = [successor, path, g, h+g,action] #added action in motion primitive 
         
          #if successor in explored:
          if any(np.array_equal(successor, explored_state) for explored_state in explored):

              print("present in explored")
              continue      
          
          #Check if duplicate node exists in fringe
          flag_do_not_append = False
          
          for node in fringe:  
              
              
              k= abs(node[0][0]-successor[0]) + abs(node[0][1]-successor[1]) 
              
              if k < 5: 
                     
              #if node[0] == successor:                  
                   #Check if existing duplicate is actually shorter path than the new node            
                  if node[2] < temp_node[2]:   # changed from <= to = 
                      
                      flag_do_not_append = True
                      #No need to check further in existing fringe
                      break
                      
          if flag_do_not_append:
              continue
          fringe.append(temp_node)  
         
  return []            
          
          
     
            
      
