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
goal_reached = False 
start_point =[0,15,0.01]
final_goal = [200,200,0]
previous_simulated_step = [ 0.001,0,0,start_point[0],start_point[1],start_point[2],0.0]
reference_path = [[0,0], [200,0], [200,200]]
ship_moving = successor_genration.matsya_mmg_wrap()
q_list = []
w_list = []
p_list=[]
pq_list =[]
i=0
while not goal_reached:
        
        path= search.aStarSearch(problem, 1, start_point, 1,[])

        if not path:

            print("No path found!")

            break
        #run the the go to goal controller  for the intermediate goal = path[0]
        #rudder input as per path[0] is  applied 
        #check the stablising time the rudder took to stablise till final value as per gtg controller
        #delay given till it reaches it goal 
        action = int(np.sign(path[1][2] - path[0][2]))*-1  #need to correct this in code 
        #print(previous_simulated_step[3],previous_simulated_step[4],"previous_simulated_step ") 
        #simulated_step = ship_moving.forwardstep(action,previous_simulated_step)
        simulated_step = ship_moving.step(action)
        #print(simulated_step[3],simulated_step[4],"simulated_step ")
        previous_simulated_step = simulated_step 
        
        start_point = [simulated_step[3],simulated_step[4],simulated_step[5]]        
        i=i+1
               

        if abs(path[0][0] - final_goal[0]) < 20 and abs(path[0][1] - final_goal[1]) < 20: 
            
            goal_reached = True
        if i==1000:
              goal_reached = True   
        

        
        print() 
        #print("final path",path ,sep="  \n     ")

        

        # Extract x, y, and orientation (in the 2D plane)
        #x = [point[0] for point in path]
        #y = [point[1] for point in path]
        orientation = [point[2] for point in path]

        
        # # Create figure and axis
        # fig, ax = plt.subplots()

        # a= [point[0] for point in reference_path]
        # b= [point[1] for point in reference_path]
        # # Plot points and connect with lines
        q=simulated_step[3]
       
        w=simulated_step[4]
        q_list.append(q)
        w_list.append(w)

        p = previous_simulated_step[3]
        pq= previous_simulated_step[4]
        p_list.append(p)
        pq_list.append(pq)


fig, ax = plt.subplots()

a= [point[0] for point in reference_path]
b= [point[1] for point in reference_path]

q= [point for point in q_list]
w= [point for point in w_list]
p=  [point for point in p_list]
pq = [point for point in pq_list] 

#ax.plot(x, y, marker ='*', linestyle = '--', color='r')
ax.plot(a,b , marker ='x', linestyle = '--',color ='b')
ax.plot(q,w , marker ='*', linestyle = '--',color ='r')
#ax.plot(p,pq , marker ='x', linestyle = '--',color ='r')
# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')

# Show plot
plt.show()       
