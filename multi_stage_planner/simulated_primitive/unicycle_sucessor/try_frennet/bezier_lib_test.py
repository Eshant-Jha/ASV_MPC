import numpy as np
from math import cos,sin,sqrt,atan
from scipy.interpolate import CubicSpline 
import matplotlib.pyplot as plt
import bezier

import numpy as np
from math import cos, sin, sqrt, atan
import matplotlib.pyplot as plt
import bezier  # Ensure you have the bezier module installed

# Bezier curve implementation

class BezierCurve:

    def __init__(self, control_points):

        self.control_points = control_points.T
        self.curve = bezier.Curve(self.control_points, degree=len(control_points) - 1)
    
    def evaluate(self, t):

        pd=[self.curve.evaluate(t)]
        return pd
    
    def derivative(self, t):
        return self.curve.evaluate_hodograph(t)

# Guidance and Agent classes
class Serret_Frenet_Guidance:
    def __init__(self, bezier_curve):
        self.bezier_curve = bezier_curve
        self.kappa = 0.1
        self.gamma = 3.0
        self.delta = 1

    def transform(self, theta):
        return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    

    def guidance_command(self, stvar):
        t, px, py, psi, U = stvar[:]
        p = np.array([[px], [py]])
        beta = 0
        
        X = psi + beta
        
        ydd = self.bezier_curve.derivative(t).item(1)
        xdd = 1
        Xt = atan(ydd / xdd)
        
        pd = np.array( [self.bezier_curve.evaluate(t)[0][0],self.bezier_curve.evaluate(t)[0][1]])
        print(pd,"bhai dekho ek baar")
        eps = self.transform(Xt).T @ (p - pd)
        s, e = eps[:]
        
        Xr = atan(-e / self.delta)
        Upp = U * cos(X - Xt) + self.gamma * s
        t_dot = Upp / sqrt(xdd**2 + ydd**2)
        
        Xd = Xt + Xr
        Ud = self.kappa * sqrt(self.delta**2 + e**2)
        Ud = 0.5098
        xd = pd[0][0]
        yd = pd[1][0]

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
    
    def simulation(self, bezier_curve, time, xspl, initlist):
        t = min(xspl)
        n = time.shape[0]
        h = time[1] - time[0]
        sol = np.zeros([3, n])
        SFG = Serret_Frenet_Guidance(bezier_curve)
        x, y, psi, u = initlist[[3, 4, 5, 0]]
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
            if abs(t - max(xspl)) < 0.5:
                sol = sol[:, :i]
                break

        return sol

# Example usage
x0 = -3
y0 = -1
psi0 = 0.0

A = Agent([x0, y0, psi0])

xspl = np.array([1, 3.5, 5, 7, 8.7, 9.5, 10, 12.2]) * 3
yspl = np.array([2, 2.5, 2.4, 1.6, 1.7, 2, 1.8, 1.2])

# Create the Bezier curve using control points
control_points = np.array([[0.0, 0.0], [1.0, 2.0], [3.0, 3.0], [4.0, 0.0]])
bezier_curve = BezierCurve(control_points)

time = np.linspace(0, 1, 100)  # Example time array
initlist = np.array([0, 0, 0, x0, y0, psi0])

sol = A.simulation(bezier_curve, time, xspl, initlist)

# Plot the results
x_ex = np.linspace(0, 1, 100)
y_ex = bezier_curve.evaluate(x_ex)[1]

plt.plot(x_ex, y_ex, label='Bezier Curve Trajectory')
plt.scatter(control_points[:, 0], control_points[:, 1], color='red', label='Control Points')
plt.legend()
plt.show()


# # Define control points
# control_points = np.array([[0.0, 0.0], [1.0, 2.0], [3.0, 3.0], [4.0, 0.0]])

# class bezier_curve():

#     def __init__(self,control_points) -> None:
#           pass
#     def curve_b(self,control_points)  :
#         # Create a Bezier curve from the control points
#         self.nodes = control_points.T
#         self.curve = bezier.Curve(self.nodes, degree=len(control_points) - 1)
#         return self.curve

#     def point_at_t(self,t):
        
#         # Evaluate the Bezier curve at multiple points
#         t_values = np.linspace(0, 1, 100)
#         #curve_points = curve.evaluate_multi(t_values).T
#         self.point_at_t = self.curve.evaluate(t)
        
#         return self.point_at_t




# # # Get coordinates of the curve at a specific t value
# # t = 0.5
# # point = curve.evaluate(t)
# # print(point,"this is  the poingt ")
# # print(f"Point on the curve at t={t}: {point.flatten()}")




# # # Plot the Bezier curve
# # plt.plot(curve_points[:, 0], curve_points[:, 1], label='Bezier Curve')
# # plt.scatter(control_points[:, 0], control_points[:, 1], color='red', label='Control Points')
# # plt.legend()
# # plt.xlabel('X')
# # plt.ylabel('Y')
# # plt.title('Bezier Curve using bezier library')
# # plt.show()