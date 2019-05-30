# Ball on Beam Parameter File
import numpy as np
# import control as cnt

# Physical parameters of the  ballbeam known to the controller
m1 = 0.0027  # Mass of the ball, kg
# m2 = 2  # mass of beam, kg
platform = 0.1016  # length of beam, m
g = 9.81  # gravity at sea level, m/s^2

# parameters for animation
radius = 0.02  # radius of ball

# Initial Conditions
x0 = 0.0  # initial ball position,m
xdot0 = 0.0
y0 = 0.0
ydot0 = 0.0
# theta0 = 0.0*np.pi/180  # initial beam angle,rads
# zdot0 = 0.0  # initial speed of ball along beam, m/s
# thetadot0 = 0.0  # initial angular speed of theh beam,rads/s

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 50  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.03  # the plotting and animation is updated at this rate

# saturation limits
Thmax = 30.*np.pi/180.
Phmax = 30.*np.pi/180.
# Fmax = 15.0  # Max Force, N

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain
