import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
from signalGenerator import signalGenerator
from ballbeamAnimation import ballbeamAnimation
from ballbeamTopAnimation import ballbeamTopAnimation
from plotData import plotData

# instantiate reference input classes
x_reference = signalGenerator(amplitude=0.5, frequency=0.1)
y_reference = signalGenerator(amplitude=0.5, frequency=0.1)
xRef = signalGenerator(amplitude=0.5, frequency=0.1)
yRef = signalGenerator(amplitude=0.5, frequency=0.1)
thetaRef = signalGenerator(amplitude=2.0*np.pi/4.0, frequency=0.1) # angle about y causes x motion
phiRef = signalGenerator(amplitude=2.0*np.pi/4.0, frequency=0.1) # angle about x causes y motion

# instantiate the simulation plots and animation
dataPlot = plotData()
x_animation = ballbeamAnimation(title="x axis")
y_animation = ballbeamAnimation(title="y axis")
top_animation = ballbeamTopAnimation(title="top view")

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop

    # set variables
    xr = x_reference.square(t)
    yr = y_reference.square(t)
    x = xRef.sin(t)
    y = yRef.cos(t)
    theta = thetaRef.sin(t)
    phi = phiRef.cos(t)

    # update animation
    state = [x[0], 0.0, y[0], 0.0]
    x_animation.drawBallbeam(state, 0, theta)
    y_animation.drawBallbeam(state, 2, phi)
    top_animation.drawBallbeamTop(state)
    dataPlot.updatePlots(t, [xr, yr], state, [theta, phi])

    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
