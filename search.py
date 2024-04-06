# -*- coding: utf-8 -*-
"""
Created on Thu Mar 14 05:15:29 2024

@author: ESHANT
@efficiently edited by: THE GREAT DON
"""
import numpy as np 
import operator

import matplotlib.pyplot as plt 
import map_grid
from matplotlib.colors import LinearSegmentedColormap


heusritic_weight = 1
break_len =5

break_dub_len = 5


def heuristic_1(problem, state, heusritic_weight, robot_id):
    
    """
    Manhattan distance
    """
    goal = problem.getGoalState(robot_id)
    del_x_1 = abs(state[0] - goal[0])
    del_y_1 = abs(state[1] - goal[1])

    ecl  = ((del_x_1)**2 + (del_y_1)**2)**0.5
    mhtn = del_x_1 + del_y_1
    
    return heusritic_weight* ecl   # take eucledian or manhanten




def aStarSearch(problem, robot_id, start_state, planner, collison_point, neighbor_robot_point, goal_states,iter):

    
  "Search the node that has the lowest combined cost and weighted heuristic first."
  #Create explored list to store popped nodes
  explored = []
  #Create Fringe to store all nodes to be expaned
  fringe = []
  path_coordinates = []
  #Add the start state to Fringe [start_state ]
  fringe.append([start_state, [start_state], 0, (heuristic_1(problem, start_state, heusritic_weight, robot_id)),[]]) #[] added in motion primitive to store action
  
 
  # print("Planning...")  
  
  
  while len(fringe)>0:
      
      fringe = sorted(fringe, key = operator.itemgetter(3))
      
      #Pop least cost node and add to explored list
      popped = fringe.pop(0)

      explored.append(popped[0]) # only the state needs to be added to explored list 
      
      # if len(explored)>iter:
      #   return  path_coordinates  ,explored, fringe
      #   break


      
      if planner==1:
          if problem.isGoalState(robot_id ,popped[0]):          
               path_coordinates = popped[1]

               return path_coordinates , explored, fringe


      if planner==1:

        successors = problem.getSuccessors(popped[0])  

        
      for successor, action, cost in successors:
          
        g = popped[2] + cost
        h = heuristic_1(problem, successor, heusritic_weight, robot_id)
        path = popped[1] + [successor]
        # temp_node = [successor, path, g, h+g] 
        temp_node = [successor, path, g, h+g,action] #added action in motion primitive 
      
        flag_do_not_append = False
        # print(explored,'/n')
      


        for explored_node in explored:  
          del_x_c = explored_node[0] - successor[0]
          del_y_c = explored_node[1] - successor[1]
          K_ecl  = ((del_x_c)**2 + (del_y_c)**2)**0.5
          psi_diff = abs(explored_node[2] - successor[2])


          if K_ecl < 2.3 :
            flag_do_not_append = True 
            continue


        for fringe_node in fringe:  
            del_x_1 = fringe_node[0][0] - successor[0]
            del_y_1 = fringe_node[0][1] - successor[1]
            K_ecl  = ((del_x_1)**2 + (del_y_1)**2)**0.5
            psi_diff = abs(fringe_node[0][2] - successor[2])

            if K_ecl < 0.58 and psi_diff <2: 
              if fringe_node[2] <= temp_node[2]:
                flag_do_not_append = True

                break
              
                    
        if flag_do_not_append:
            #In this case do not add the new node 
            continue
        
        #If none of the above then add successor to fringe 
        
        fringe.append(temp_node)

          

     
  print("mayday A* killed by you")
  return explored ,explored, fringe

