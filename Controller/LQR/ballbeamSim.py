import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory

import ballbeamParam as P

from ballbeamDynamics import ballbeamDynamics
from ballbeamController import ballbeamController
from signalGenerator import signalGenerator
from ballbeamAnimation import ballbeamAnimation
from plotData import plotData
from plotObserverData import plotObserverData
from ballbeamTopAnimation import ballbeamTopAnimation

# Hardware = False
Hardware = True
if Hardware:
    # Arduino Serial libraries
    from serial import Serial
    from time import sleep
    port = "/dev/ttyUSB0"
    sleep(1)
    ser = Serial(port,19200)
    sleep(4)

Camera = False
# Camera = True
if Camera:
    from BallPosition import BallPosition
    BP = BallPosition()
    center = BP.Position("green")
    xy = np.asarray(BP.Position("orange"))
    ctrl = ballbeamController(xy[0],xy[1])
else:
    ctrl = ballbeamController(P.x0,P.y0)
    # ctrl = ballbeamController()

# instantiate ballbeam, controller, and reference classes
ballbeam = ballbeamDynamics()

if Camera:
    x_reference = signalGenerator(amplitude=0., frequency=0.2)
    y_reference = signalGenerator(amplitude=0., frequency=0.2,t_offset=True)
else:
    x_reference = signalGenerator(amplitude=0.05, frequency=0.275)
    y_reference = signalGenerator(amplitude=0.05, frequency=0.275,t_offset=True)

# instantiate the simulation plots and animation
dataPlot = plotData()
# x_animation = ballbeamAnimation(title="x axis")
# y_animation = ballbeamAnimation(title="y axis")
top_animation = ballbeamTopAnimation(title="top view")
# observerPlot = plotObserverData()

# LQR stuff
# K = np.array([[-1.2884, -0.5290, 0., 0.],[0., 0., -1.2884, -0.5290]])
# x = P.x0
# x_dot = P.xdot0
# x_d1 = P.x0
# y = P.y0
# y_dot = P.ydot0
# y_d1 = P.y0
# state_hat = np.array([[x, x_dot, y, y_dot]]).T
# limit = np.pi/6

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    x_ref = x_reference.square(t)
    y_ref = y_reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate

        state_r = np.array([[x_ref[0], y_ref[0]]]).T
        if Camera:
            point = BP.Position("orange")
            xy = np.array([[point[0],point[1]]]).T
            print(xy)
        else:
            point = ballbeam.outputs()
            xy = np.array([[point[0],point[1]]]).T

        print("req",state_r,"\nxy",xy)
        u = ctrl.u(state_r,xy)
        ballbeam.propagateDynamics(u)#input)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts

        ## Serial stuff
        if Hardware:
            # ser.flush()
            mult = 30.
            com = str(int((-u[0]*mult+90.)/180.*(547.*2.)+1000)) + "," + str(int((-u[1]*mult+90.)/180.*(415.*2.)+1000)) + "\n"
            ser.write(com.encode())
            print(com)
            # print(t)
            # msg = ser.readline()
            # print("Message from arduino:",msg)
            # # sleep(0.007)

    # update animation and data plots
    # x_animation.drawBallbeam(ballbeam.states(), 0, u[0])
    # y_animation.drawBallbeam(ballbeam.states(), 2, u[1])
    state = np.array([xy[0],0.,xy[1],0.])
    top_animation.drawBallbeamTop(state)
    input_ref = [x_ref, y_ref]
    dataPlot.updatePlots(t, input_ref,state, u)
    # top_animation.drawBallbeamTop(ballbeam.states())
    # input_ref = [x_ref, y_ref]
    # dataPlot.updatePlots(t, input_ref, ballbeam.states(), u)
    # observerPlot.updatePlots(t, ballbeam.states(), ctrl.z_hat)
    plt.pause(0.001)  # the pause causes the figure to be displayed during the simulation

if Camera:
    BP.shutdown()

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
