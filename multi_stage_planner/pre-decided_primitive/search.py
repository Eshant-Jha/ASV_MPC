# -*- coding: utf-8 -*-
"""
Created on Thu Mar 14 05:15:29 2024

@author: ESHANT
"""

import numpy as np 
import operator
from  testing import Cross_track
import matplotlib.pyplot as plt 

reference_path = [[0,0], [200,0], [200,200]]

def heuristic_1(problem,state,refrence_path):
    
    cte   = Cross_track()
    
    error = cte.cross_track_error(reference_path,state[0:2])
    
    return error
    
def aStarSearch(problem, robot_id, start_state, planner, goal_states):

    
  "Search the node that has the lowest combined cost and weighted heuristic first."
  #Create explored list to store popped nodes
  explored = []
  #Create Fringe to store all nodes to be expaned
  fringe = []
  #Add the start state to Fringe [start_state ]
  fringe.append([start_state, [start_state], 0, (heuristic_1(problem, start_state, []) ),[]]) #[] added in motion primitive to store action
  
 
  print("Planning...")  
  
  
  while len(fringe)>0:
      
      fringe = sorted(fringe, key = operator.itemgetter(3))
      
      #Pop least cost node and add to explored list
      current_node = fringe.pop(0)
    
      explored.append(current_node[0]) # only the state needs to be added to explored list 
      
     
      
      if planner==1:
          #if problem.isGoalState(robot_id ,current_node[0]): 
              #planning only till pre-defined  number of steps 
              steps= 2        
              path_coordinates = current_node[1]
              if len(path_coordinates) > steps :
               
                return path_coordinates
     

      #Expand node and get successors
      if planner==1:  #if its planning globally

        successors = problem.getSuccessors(current_node[0])  
        
      for successor, action, cost in successors:
          
          g = current_node[2] + cost
          
          g=0
          
          h = heuristic_1(problem, successor, reference_path)

               
          
          path = current_node[1] + [successor]
          temp_node = [successor, path, g, h+g] 
          temp_node = [successor, path, g, h+g,action] #added action in motion primitive 
         
          #Check if the successor already exists in explored list
          print(successor,explored,"idhar dekhte hai ")
          if successor in explored:
              continue       #If so already optimal do not add to list
          
          #Check if duplicate node exists in fringe
          flag_do_not_append = False
          
          for node in fringe:  
              #print(node[0][2],"popped out node heading angle " ,)
              print(node[0],"node hai ")
              k= abs(node[0][0]-successor[0]) + abs(node[0][1]-successor[1]) + abs(node[0][2]-successor[2])
              
              if k < 0.2: 
                     
              #if node[0] == successor:                  
                   #Check if existing duplicate is actually shorter path than the new node            
                  if node[2] <= temp_node[2]:
                      
                      flag_do_not_append = True
                      
                      #No need to check further in existing fringe
                      break
                      
          if flag_do_not_append:
             
              continue
          
          #If none of the above then add successor to fringe 
          
          fringe.append(temp_node)
          #print(f"Current node: {current_node}, Fringe size: {len(fringe)}")
          
  return []            
          
          
     
            
      
