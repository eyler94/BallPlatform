import numpy as np
import ballbeamParam as P
import scipy.linalg

printWide = True
if printWide:
    import pandas as pd

    pd.set_option('display.width', 320)
    pd.set_option('display.max_columns', 12)
    np.set_printoptions(linewidth=320)


class ballbeamController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self, x0, y0):
        # PID Terms
        self.Kp = -2.5  # my original git value was -2
        self.Kd = -1  # -2# my original git value was -1

        self.state = np.array([x0, 0., y0, 0.]).T
        self.state_d1 = np.array([x0, 0., y0, 0.]).T

        # Integration stuff
        self.Ki = 0.0#001#1
        self.error_state = np.zeros((2,))
        self.int_error = np.zeros((2,))
        self.error_d1 = np.zeros((2,))

        # Saturation protection
        self.Ts = P.Ts
        self.limit = np.pi / 20.

        # Output Smoothing
        self.first = True
        self.length = 3
        self.state_Array = np.ones((2, self.length))

        self.u_prev = np.zeros((2,))
        self.u_diff = np.zeros((2,))

    def u(self, req, cur):
        self.state[0] = cur[0]
        self.state[2] = cur[1]

        self.derivatives()

        self.error_state = req - cur

        g = 9.81
        epsilon = 0.1

        u_unsat = (np.abs(self.state[1::2]/g)+np.abs(self.u_diff**2*self.state[0::2]/g) + 0.1)*self.sliding_saturation((self.state[1::2]+self.state[0::2])/epsilon)
        # u_unsat = -(np.abs(self.state[1::2]/g)+np.abs(self.u_diff**2*self.error_state/g) + 0.1)*self.sliding_saturation((self.state[1::2]+self.error_state)/epsilon)

        deriv_lim = 0.2
        if self.state[1] <= deriv_lim:
            self.integrateError(self.error_state, 0)
        if self.state[3] <= deriv_lim:
            self.integrateError(self.error_state, 1)
        u_unsat = u_unsat - self.Ki * self.int_error.T
        u_sat = self.saturate(u_unsat)
        self.integrator_anti_windup(u_sat, u_unsat)

        # u_unsat = -(np.abs(self.state[1::2]/g)+np.abs(self.u_diff*self.error_state/g) + 0.1)*np.sign(self.state[1::2]+self.error_state)
        u_sat = self.saturate(u_unsat)
        self.u_diff = (u_sat-self.u_prev)/self.Ts
        self.u_prev = u_sat

        return u_sat

    def derivatives(self):
        # print("state",self.state,"\nprev state:",self.state_d1)
        if self.first:
            self.state_Array[0, :] = self.state_Array[0, :] * self.state[0].copy()
            self.state_Array[1, :] = self.state_Array[1, :] * self.state[2].copy()
            self.state[1] = 0.
            self.state[3] = 0.
            self.first = False
        else:
            for i in range(self.length - 1, 0, -1):
                self.state_Array[0][i] = self.state_Array[0][i - 1].copy()
                self.state_Array[1][i] = self.state_Array[1][i - 1].copy()
            self.state_Array[0] = self.state[0]
            self.state_Array[1] = self.state[2]
            self.state[1] = (self.state[0] - self.state_d1[0]) / self.Ts
            self.state[3] = (self.state[2] - self.state_d1[2]) / self.Ts

        self.state[0] = np.average(self.state_Array[0, :])
        self.state[2] = np.average(self.state_Array[1, :])
        self.state_d1[0] = self.state[0]
        self.state_d1[2] = self.state[2]

    def integrateError(self, error, spot):
        self.int_error[spot] = self.int_error[spot] + (self.Ts / 2.) * (error[spot] + self.error_d1[spot])
        self.error_d1[spot] = error[spot]

    def saturate(self, u_unsat):
        u_sat = np.array([0.,0.])
        for spot in range(0, 2):
            if np.abs(u_unsat[spot]) >= self.limit:
                u_sat[spot] = np.sign(u_unsat[spot])*self.limit
            else:
                u_sat[spot] = u_unsat[spot]
        return u_sat

    def integrator_anti_windup(self, u_sat, u_unsat):
        if self.Ki != 0:
            self.int_error = self.int_error + self.Ts / self.Ki * (u_sat - u_unsat)

    def sliding_saturation(self, frac):
        frac_sat = np.zeros((2,))
        for spot, value in enumerate(frac):
            if np.abs(value) > 1:
                frac_sat[spot] = np.sign(value)
            else:
                frac_sat[spot] = value
        return frac_sat

