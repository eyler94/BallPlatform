import numpy as np
import ballbeamParam as P
# import ballbeamParamLQR as PLQR

class ballbeamController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.K = np.array([[-1.2884, -0.5290, 0., 0.],[0., 0., -1.2884, -0.5290]])
        x = P.x0
        self.x_dot = P.xdot0
        self.x_d1 = P.x0
        y = P.y0
        self.y_dot = P.ydot0
        self.y_d1 = P.y0
        self.state_hat = np.array([[x, self.x_dot, y, self.y_dot]]).T
        self.limit = np.pi/6

        ##integrator stuff
        self.Th_d1 = 0.0  # Computed Force, delayed by one sample
        self.Ph_d1 = 0.0
        self.integrator_x = 0.0        # integrator
        self.integrator_y = 0.0        # integrator
        self.error_x_d1 = 0.0          # error signal delayed by 1 sample
        self.error_y_d1 = 0.0          # error signal delayed by 1 sample
        self.kxi = 0.5            # Integral gain
        self.kyi = 0.5             # Integral gain

    def u(self, state_r, state_c):
        x = state_c[0]
        self.x_dot = (x-self.x_d1)/P.Ts
        self.x_d1 = x
        y = state_c[1]
        self.y_dot = (y-self.y_d1)/P.Ts
        self.y_d1 = y

        #integrate error
        error_x = state_r[0] - x
        error_y = state_r[1] - y
        self.integrateError(error_x, error_y)

        state_hat = np.array([[x, self.x_dot, y, self.y_dot]]).T
        u = -state_r-self.K@state_hat
        theta_unsat = u[0] - self.kxi*self.integrator_x
        phi_unsat = u[1] - self.kyi*self.integrator_y
        theta = self.saturate(theta_unsat,self.limit)
        phi = self.saturate(phi_unsat,self.limit)
        self.integrator_x_AntiWindup(theta, theta_unsat)
        self.integrator_y_AntiWindup(phi, phi_unsat)
        u[0] = theta
        u[1] = phi
        return u

    def integrator_x_AntiWindup(self, Theta, Theta_unsat):
        # integrator anti - windup
        if self.kxi != 0.0:
            self.integrator_x = self.integrator_x + P.Ts/self.kxi*(Theta-Theta_unsat)

    def integrator_y_AntiWindup(self, Phi, Phi_unsat):
        # integrator anti - windup
        if self.kyi != 0.0:
            self.integrator_y = self.integrator_y + P.Ts/self.kyi*(Phi-Phi_unsat)

    def integrateError(self, error_x, error_y):
        self.integrator_x = self.integrator_x + (P.Ts/2.0)*(error_x + self.error_x_d1)
        self.integrator_y = self.integrator_y + (P.Ts/2.0)*(error_y + self.error_y_d1)
        self.error_x_d1 = error_x
        self.error_y_d1 = error_y

    def saturate(self,u,limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u
