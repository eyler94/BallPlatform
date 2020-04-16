import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory

import ballbeamParam as P

from ballbeamDynamics import ballbeamDynamics
# from ballbeamController import ballbeamController
# from ballbeamControllerLinearFeedback import ballbeamController
from ballbeamControllerSlidingMode import ballbeamController
from signalGenerator import signalGenerator
from plotData import plotData
from ballbeamTopAnimation import ballbeamTopAnimation

Hardware = False
Hardware = True
if Hardware:
    # Arduino Serial libraries
    from serial import Serial
    from time import sleep
    port = "/dev/ttyUSB0"
    sleep(1)
    ser = Serial(port,19200)
    sleep(2)

Camera = False
Camera = True
if Camera:
    from BallPosition import BallPosition
    BP = BallPosition(2)
    # center = BP.Position("green")
    xy = np.asarray(BP.Position("orange"))
    ctrl = ballbeamController(xy[0], xy[1])
else:
    ctrl = ballbeamController(P.x0, P.y0)
    # ctrl = ballbeamController()

# instantiate ballbeam, controller, and reference classes
ballbeam = ballbeamDynamics()

if Camera:
    x_reference = signalGenerator(amplitude=0.0, frequency=0.051)
    y_reference = signalGenerator(amplitude=0.0, frequency=0.051)#, t_offset=True)
    # x_reference = signalGenerator(amplitude=0.0325, frequency=0.051)
    # y_reference = signalGenerator(amplitude=0.0325, frequency=0.051)#, t_offset=True)
else:
    x_reference = signalGenerator(amplitude=0.0, frequency=0.051)
    y_reference = signalGenerator(amplitude=0.0, frequency=0.051)#, t_offset=True)
    # x_reference = signalGenerator(amplitude=0.0325, frequency=0.051)
    # y_reference = signalGenerator(amplitude=0.0325, frequency=0.051)#, t_offset=True)

# instantiate the simulation plots and animation
dataPlot = plotData()
# TopData = plotTopData()
top_animation = ballbeamTopAnimation(title="top view")

t = P.t_start  # time starts at t_start
x = []
y = []
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # x_ref = x_reference.square(t)
    # y_ref = y_reference.square(t)
    x_ref = x_reference.sin(t)
    y_ref = y_reference.cos(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate

        state_r = np.array([x_ref[0], y_ref[0]]).T
        if Camera:
            point = BP.Position("orange")
            xy = np.array([point[0], point[1]]).T
            print("x,y:", xy)
        else:
            point = ballbeam.outputs()
            xy = np.array([point[0], point[1]]).T
            print(xy)

        u = ctrl.u(state_r, xy)
        ballbeam.propagateDynamics(u)#input)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts

        ## Serial stuff
        if Hardware:
            # ser.flush()
            print("u:", u)
            com = str(int((-u[0]+np.pi/2.)/np.pi*(550.*2.)+1000)) + "," + str(int((-u[1]+np.pi/2.)/np.pi*(400.*2.)+1000)) + "\n"
            ser.write(com.encode())
            print(com)

    # update animation and data plots
    x.append(xy[0])
    y.append(xy[1])
    state = np.array([xy[0], 0., xy[1], 0.])
    top_animation.drawBallbeamTop(state)
    input_ref = [x_ref, y_ref]
    dataPlot.updatePlots(t, input_ref, state, u)
    # TopData.updatePlots(t, input_ref,state, u)
    plt.pause(0.00001)  # the pause causes the figure to be displayed during the simulation

if Camera:
    BP.shutdown()

np.save("x_LF.npy", x)
np.save("y_LF.npy", y)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
