import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import ballbeamParam as P
from ballbeamDynamics import ballbeamDynamics
from ballbeamController import ballbeamController
from signalGenerator import signalGenerator
from ballbeamAnimation import ballbeamAnimation
from plotData import plotData
from plotObserverData import plotObserverData
from ballbeamTopAnimation import ballbeamTopAnimation

# instantiate ballbeam, controller, and reference classes
ballbeam = ballbeamDynamics()
ctrl = ballbeamController()
x_reference = signalGenerator(amplitude=0.001, frequency=0.02)
y_reference = signalGenerator(amplitude=0., frequency=0.02)

# instantiate the simulation plots and animation
dataPlot = plotData()
x_animation = ballbeamAnimation(title="x axis")
y_animation = ballbeamAnimation(title="y axis")
top_animation = ballbeamTopAnimation(title="top view")
observerPlot = plotObserverData()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    x_ref = x_reference.square(t)
    y_ref = y_reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        u = ctrl.u(x_ref[0], y_ref[0], ballbeam.outputs())  # Calculate the control value
        ballbeam.propagateDynamics(u)#input)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    x_animation.drawBallbeam(ballbeam.states(), 0, u[0])
    y_animation.drawBallbeam(ballbeam.states(), 2, u[1])
    top_animation.drawBallbeamTop(ballbeam.states())
    input_ref = [x_reference.square(t), y_reference.square(t)]
    dataPlot.updatePlots(t, input_ref, ballbeam.states(), u)
    observerPlot.updatePlots(t, ballbeam.states(), ctrl.z_hat)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
