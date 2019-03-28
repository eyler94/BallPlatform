import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
from signalGenerator import signalGenerator
from ballbeamAnimation import ballbeamAnimation
from plotData import plotData
from ballbeamDynamics import ballbeamDynamics
from ballbeamTopAnimation import ballbeamTopAnimation

# instantiate ballbeam, controller, and reference classes
ballbeam = ballbeamDynamics()
x_reference = signalGenerator(amplitude=0.5, frequency=0.02)
y_reference = signalGenerator(amplitude=0.5, frequency=0.02)
theta = signalGenerator(amplitude=.1, frequency=1)
thdot = 0.
phi = signalGenerator(amplitude=.1, frequency=1)
phdot = 0.

# instantiate the simulation plots and animation
dataPlot = plotData()
x_animation = ballbeamAnimation(title="x axis")
y_animation = ballbeamAnimation(title="y axis")
top_animation = ballbeamTopAnimation(title="top view")

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = [x_reference.square(t),y_reference.square(t)]
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        th = theta.nsin(t)
        thdot = (th[0]-theta.nsin(t-P.Ts))/P.Ts
        ph = phi.nsin(t)
        phdot = (ph[0]-phi.nsin(t-P.Ts))/P.Ts
        ballbeam.propagateDynamics([th[0],ph[0]])  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    x_animation.drawBallbeam(ballbeam.states(), 0, th)
    y_animation.drawBallbeam(ballbeam.states(), 2, ph)
    top_animation.drawBallbeamTop(ballbeam.states())
    dataPlot.updatePlots(t, ref_input, ballbeam.states(), [th[0], ph[0]])
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from clonsing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
