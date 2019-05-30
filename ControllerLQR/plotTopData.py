import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing


class plotTopData:
    '''
        This class plots the time histories for the pendulum data.
    '''

    def __init__(self):
        # Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 1    # Number of subplot rows
        self.num_cols = 1    # Number of subplot columns

        # Crete figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)

        # Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.xref_history = []  # reference position x_r
        self.x_history = []  # position x
        self.theta_history = []  # angle theta
        self.yref_history = []  # reference position x_r
        self.y_history = []  # position x
        self.phi_history = []  # angle theta

        # create a handle for every subplot.
        self.handle = []
        self.handle.append(myPlot(self.ax, xlabel='x', ylabel='y', title='Ball on Beam Data'))

    def updatePlots(self, t, reference, states, ctrl):
        '''
            Add to the time and data histories, and update the plots.
        '''
        # print(ctrl[0][0])
        # update the time history of all plot variables
        self.time_history.append(t)  # time
        self.xref_history.append(reference[0])  # reference base position
        self.x_history.append(states[0])  # base position
        self.theta_history.append(180.0/np.pi*ctrl[0])  # rod angle (converted to degrees)
        self.yref_history.append(reference[1])  # reference base position
        self.y_history.append(states[2])  # base position
        self.phi_history.append(180.0/np.pi*ctrl[1])  # rod angle (converted to degrees)

        # update the plots with associated histories
        self.handle[0].updatePlot([self.x_history, self.xref_history], [self.y_history, self.yref_history])

class myPlot:
    '''
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        '''
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data.
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True

    def updatePlot(self, x_data, y_data):
        '''
            Adds y_data to the plot.
            x_data is a list,
            y_data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first x_data routine is called
            for i in range(len(y_data)):
                # Instantiate line object and add it to the axes
                self.line.append(Line2D(x_data[i],
                                        y_data[i],
                                        color=self.colors[np.mod(i, len(self.colors) - 1)],
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=self.legend if self.legend != None else None))
                self.ax.add_line(self.line[i])
            self.init = False
            # add legend if one is specified
            if self.legend != None:
                plt.legend(handles=self.line)
        else: # Add new y_data to the plot
            # Updates the x and y y_data of each line.
            for i in range(len(self.line)):
                self.line[i].set_yy_data(x_data[i])
                self.line[i].set_yy_data(y_data[i])

        # Adjusts the axis to fit all of the y_data
        self.ax.relim()
        self.ax.autoscale()
