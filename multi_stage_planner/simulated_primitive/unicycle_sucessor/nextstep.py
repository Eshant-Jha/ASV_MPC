import numpy as np
from testing import Cross_track

class Arena:
   
   arena_map = [] 

   
   def __init__(self, id ,heading_angle):
       self.id =id
       self.heading_angle= heading_angle
     
   def getSuccessors(self,state):

    #import pdb; pdb.set_trace()
    reference_path = [[0,0], [200,0], [200,200]]
    successors = []
    x, y, theta = state[0], state[1], state[2]
    
    # Define the possible omega values
    omegas = [0, -1, 1]
    
    # Time step for discretization
    dt = 5#10  # you can adjust the time step as needed
    
    for omega in omegas:
        # Calculate the next state
        angle_factor = 0.07
        ang = theta + omega * dt*0.05
        new_theta = (ang + np.pi) % (2 * np.pi) - np.pi
        new_x = x + np.cos(new_theta) * dt
        new_y = y + np.sin(new_theta) * dt
        
        # Next state
        next_state = [new_x, new_y, new_theta]
        
        # Calculate the cost
        cte   =  Cross_track()
        error =  cte.cross_track_error(reference_path,next_state[0:2])  
        cost =   error  # the cost can be adjusted based on your application
        
        # Action corresponding to the omega value
        action = omega*30
        
        # Add the successor to the list
        successors.append((next_state, action, cost))
    
    return successors
   
   def isGoalState(self,state,reference_path):
       
       goal= reference_path[-1]
       threshold= abs(goal[0]- state[0]) +abs(goal[1] - state[1])
       if threshold < 4 :
          
          return True 
       
       return False
   



       