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
x_reference = signalGenerator(amplitude=0.5, frequency=0.02)
y_reference = signalGenerator(amplitude=0.5, frequency=0.02)

# instantiate the simulation plots and animation
dataPlot = plotData()
x_animation = ballbeamAnimation(title="x axis")
y_animation = ballbeamAnimation(title="y axis")
top_animation = ballbeamTopAnimation(title="top view")
observerPlot = plotObserverData()

# set disturbance input
disturbance = 1.0

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = [x_reference.square(t),y_reference.square(t)]
    # ref_input[0] += 0.25
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, ballbeam.outputs())  # Calculate the control value
        input = [u[0]+disturbance]
        ballbeam.propagateDynamics(input)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    # animation.drawBallbeam(ballbeam.states())
    # dataPlot.updatePlots(t, ref_input, ballbeam.states(), u)
    x_animation.drawBallbeam(ballbeam.states(), 0, th)
    y_animation.drawBallbeam(ballbeam.states(), 2, ph)
    top_animation.drawBallbeamTop(ballbeam.states())
    dataPlot.updatePlots(t, ref_input, ballbeam.states(), [th, ph])
    observerPlot.updatePlots(t, ballbeam.states(), ctrl.x_hat)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
