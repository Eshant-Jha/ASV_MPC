import numpy as np
from math import cos, sin, sqrt, atan2
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import itertools

class Serret_Frenet_Guidance:

    def __init__(self, spline_x, spline_y,params):

        self.spline_x = spline_x
        self.spline_y = spline_y
        self.kappa = params[0] #0.1
        self.gamma = params[1] #3.0
        self.delta = params[2] #10  # look-ahead distance

    def transform(self, theta):
        return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

    def guidance_command(self, stvar):
        t, px, py, psi, U = stvar[:]
        p = np.array([[px], [py]])
        beta = 0

        X = psi + beta

        # Get derivatives for x and y from the spline
        xdd = self.spline_x(t, 1).item(0)  # derivative of x with respect to t
        ydd = self.spline_y(t, 1).item(0)  # derivative of y with respect to t
        Xt = atan2(ydd, xdd)

        pd = np.array([[self.spline_x(t).item(0)], [self.spline_y(t).item(0)]])
        eps = self.transform(Xt).T @ (p - pd)
        s, e = eps[:]  # s is along track error and e is cross-track error

        Xr = atan2(-e, self.delta)
        Upp = U * cos(X - Xt) + self.gamma * s
        print(Upp,"path tangential speed of the point on curve ")
        t_dot = Upp / sqrt(xdd ** 2 + ydd ** 2)  # rate of change of point on curve

        Xd = Xt + Xr
        Ud = self.kappa * sqrt(self.delta ** 2 + e ** 2) 

        #limiting speed of vessel as per constraint
        if Ud > 20:
            Ud=20
        print(Ud,"desired speed of vessel chosen")

        return [Xd, Ud, t_dot[0]]

class Agent:

    def __init__(self, init):
        self.x0 = init[0]
        self.y0 = init[1]
        self.psi0 = init[2]

    def initial(self):
        return [self.x0, self.y0, self.psi0]

    def dynamics(self, states, dstate, h):
        self.x = states[0]
        self.y = states[1]

        self.psid = dstate[1]
        self.ud = dstate[0]

        stder = np.zeros(2)
        stder[0] = self.ud * np.cos(self.psid)
        stder[1] = self.ud * np.sin(self.psid)
        stnew = np.array([self.psid, stder[0] * h + self.x, stder[1] * h + self.y])

        return stnew

    def simulation(self, spline_x, spline_y, time, t_param, initial,params):
        
        t = min(t_param)
        n = time.shape[0]
        h = time[1] - time[0]
        sol = np.zeros([3, n])
        SFG = Serret_Frenet_Guidance(spline_x, spline_y,params)
        print(SFG.gamma)
        x, y, psi, u = initial[0], initial[1], initial[2], initial[3]
        var0 = [t, x, y, psi, u]
        i = 0

        while i < n:
            guide = SFG.guidance_command(var0)
            Xd, Ud, td = guide[:]
            sol[:, i] = x, y, Xd
            statenew = self.dynamics([x, y], [Ud, Xd], h)
            psi, x, y = statenew[:]
            t = td * h + t    
            var0 = [t, x, y, psi, Ud]
            i += 1
            if t > max(t_param):
                sol = sol[:, :i]
                break

        return sol

# Example with arbitrary x and y coordinates
xspl = np.array([0, 6, 6, 8, 10])
yspl = np.array([0, 0, 5, 8, 5])

# Generate a parameter t that is strictly increasing
t_param = np.linspace(0, 1, len(xspl))

# Create cubic splines for the parametric curves
spline_x = CubicSpline(t_param, xspl)#Suppose t_param = [0, 0.25, 0.5, 0.75, 1] and xspl = [0, 2, 4, 6, 8].The function CubicSpline(t_param, xspl) will create a smooth curve that passes through these points at the specified t_param values.
spline_y = CubicSpline(t_param, yspl)

# Generating the reference path (high-resolution)
t_high_res = np.linspace(0, 1, 1000)
x_ex = spline_x(t_high_res)
y_ex = spline_y(t_high_res)

# Simulation parameters
T = 45
time = np.linspace(0, T, 500)
x0 = xspl[0]  # Starting point on the path
y0 = yspl[0]
psi0 = 0.0

A = Agent([x0, y0, psi0])
X0 = [t_param[0], x0, y0, psi0, 1.0]  # Initial state
params=[5.1 ,11,1]
Refr1 = A.simulation(spline_x, spline_y, time, t_param, np.array(X0),params)

def calculate_total_cte(solution, reference_x, reference_y):
    total_cte = 0
    for i in range(solution.shape[1]):
        sim_x, sim_y = solution[0, i], solution[1, i]
        distances = np.sqrt((reference_x - sim_x) ** 2 + (reference_y - sim_y) ** 2)
        min_distance = np.min(distances)
        total_cte += min_distance
       
    return total_cte

total_cte = calculate_total_cte(Refr1, x_ex, y_ex)
# Plotting the results
plt.plot(Refr1[0, :], Refr1[1, :], label="Simulated Trajectory")
plt.plot(x_ex, y_ex, label="Reference Path")
plt.legend()
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title(f'''Total Cross-track error is {round(total_cte,3)} with ref to complete actual path ''')
plt.axis('equal')
plt.show()

#Example with arbitrary x and y coordinates
xspl = np.array([0, 6, 6, 8, 10])
yspl = np.array([0, 0, 5, 8, 5])
t_param = np.linspace(0, 1, len(xspl))
spline_x = CubicSpline(t_param, xspl)
spline_y = CubicSpline(t_param, yspl)

# Reference path for comparison
t_high_res = np.linspace(0, 1, 1000)
x_ex = spline_x(t_high_res)
y_ex = spline_y(t_high_res)

# Simulation parameters
T = 45
time = np.linspace(0, T, 500)
x0 = xspl[0]
y0 = yspl[0]
psi0 = 0.0
A = Agent([x0, y0, psi0])
X0 = [t_param[0], x0, y0, psi0, 1.0]

# Define parameter ranges as per the conmstraints of vessel
kappa_range = np.linspace(0.1, 10, 50)     
gamma_range = np.linspace(2.0, 15.0, 5)
delta_range = np.linspace(1, 15, 5)

best_score = float('inf')
best_params = None

# Define the cost function for total cross-track error


# Perform grid search
for kappa, gamma, delta in itertools.product(kappa_range, gamma_range, delta_range):
    SFG = Serret_Frenet_Guidance(spline_x, spline_y,params)
    #SFG.kappa = kappa
    #SFG.gamma = gamma
    #SFG.delta = delta
    
    params=[kappa,gamma,delta]
    # Run the simulation
    Refr1 = A.simulation(spline_x, spline_y, time, t_param, np.array(X0),params)
    
    # Calculate the total cross-track error
    total_cte = calculate_total_cte(Refr1, x_ex, y_ex)
    
    # Check if this combination is better
    if total_cte < best_score:
        best_score = total_cte
        best_params = (kappa, gamma, delta)

print(f"Best Parameters: Kappa = {best_params[0]}, Gamma = {best_params[1]}, Delta = {best_params[2]}")
print(f"Best Total Cross-Track Error: {best_score}")