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
        
        self.time_step = 0.4
        
        
        self.forward_primitive = []
        #self.time_step = 0.00466  ################how you reached this value ?

        self.delta_c = 0        
        self.sim_state=np.array([0.01, 0, 0, 0, 0, 0 , 0])#np.array([1e-6, 0, 0, 0, 0, 0 , 0])  #-np.pi/2
        
     def ssa(ang, deg=False):
        
        if deg:
            ang = (ang + 180) % (360.0) - 180.0
        else:
            ang = (ang + np.pi) % (2 * np.pi) - np.pi
        return ang
    
    
     def step(self,action):

        #manually added timestep here again 
        self.time_step = 0.04
        tspan = (0, self.time_step)
        yinit = self.sim_state
        print(yinit,"initial position now in step function ")
        #using i deciding 
        
        action_to_delta_c = { 0: 0, -1: -(20.0 / 180.0) * np.pi, 1: (20.0 / 180.0) * np.pi}

        self.delta_c = action_to_delta_c[action]
   
            
        #print("Rudder angle applied now",self.delta_c*(180/np.pi))
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
        psi_rad = sol.y[5, -1]     # psi
        
        psi = (psi_rad + np.pi) % (2 * np.pi) - np.pi
        
        delta = sol.y[6, -1]
        
        self.new_primitive_f = [x, y, np.rad2deg(psi), delta*(180/np.pi)]
        
        self.forward_primitive.append(self.new_primitive_f)
        self.sim_state = np.array([sol.y[0, -1], sol.y[1, -1], sol.y[2, -1], sol.y[3, -1], sol.y[4, -1], psi, sol.y[6, -1]])
        obs_state = np.array([u, v, r, x, y, psi, delta])    # This state is in NED
        
    
        self.obs_state = obs_state
        print(obs_state,"state obsered")
        #return self.new_primitive_f
        return obs_state
     def forwardstep(self,action,state):

        #self.time_step = 1
        tspan = (0, 0.04)
        
        yinit = state
        

        #using i deciding 
        

        action_to_delta_c = { 0: 0, -1: -(20.0 / 180.0) * np.pi, 1: (20.0 / 180.0) * np.pi}

        self.delta_c = action_to_delta_c[action]
   
            
        #print("Rudder angle applied now",self.delta_c*(180/np.pi))
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
        
        self.new_primitive_f = [x, y, np.rad2deg(psi), delta*(180/np.pi)]
        
        self.forward_primitive.append(self.new_primitive_f)
        self.sim_state = np.array([sol.y[0, -1], sol.y[1, -1], sol.y[2, -1], sol.y[3, -1], sol.y[4, -1], psi, sol.y[6, -1]])
        obs_state = np.array([u, v, r, x, y, psi, delta])       # This state is in NED
        
    
        self.obs_state = obs_state
        
        return obs_state
      
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


    #print("\n \nResults for Left Action (-1):")
    # for i, val in enumerate(l):
    #     print(f"\nStep {i+1}: {val}")
    
    # #print("\n \nResults for Right Action (1):")
    # for i, val in enumerate(r):
    #     print(f"\nStep {i+1}: {val}")
    
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
    
    
      

      
      
      
      
      
      