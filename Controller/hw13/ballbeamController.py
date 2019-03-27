import numpy as np
import ballbeamParam as P
import ballbeamParamHW13 as P13

class ballbeamController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.z_hat = np.matrix([
            [0.0],  # initial estimate for x_hat
            [0.0],  # initial estimate for x_dot_hat
            [0.0],  # initial estimate for y_hat
            [0.0]])  # initial estimate for y_dot_hat
        self.Th_d1 = 0.0  # Computed Force, delayed by one sample
        self.Ph_d1 = 0.0
        self.integrator_x = 0.0        # integrator
        self.integrator_y = 0.0        # integrator
        self.error_x_d1 = 0.0          # error signal delayed by 1 sample
        self.error_y_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P13.K               # state feedback gain
        self.ki = P13.ki             # Integral gain
        self.L = P13.L                 # observer gain
        self.A = P13.A                 # system model
        self.B = P13.B
        self.C = P13.C
        self.limit = P.Fmax          # Maximum force
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, r_input, c_state):
        # y_r is the referenced input
        # y is the current state
        x_r = r_input[0]
        x = c_state[0]
        y_r = r_input[2]
        y = c_state[2]

        # update the observer and extract z_hat
        self.updateObserver(c_state)
        #######################################################################
        z_hat = self.x_hat[1]

        # integrate error
        error_x = x_r - x
        error_y = y_r - y
        self.integrateError(error_x, error_y)

        # Construct the state
        ze = np.matrix([[0.0], [0.0], [0.0], [0.0]])
        x_tilde = self.x_hat - ze

        # Compute the state feedback controller
        Theta_unsat = -self.K*x_tilde - self.ki*self.integrator_x
        Phi_unsat = -self.K*x_tilde - self.ki*self.integrator_y

        F_unsat = F_tilde
        F = self.saturate(F_unsat)
        self.integratorAntiWindup(F, F_unsat)
        self.updateForce(Th, Ph)
        return [F.item(0)]

    def updateObserver(self, y_m):
        N = 10
        y = np.matrix([
            [y_m[1]],
            [y_m[0]]])
        xe = np.matrix([[0.0], [P.ze], [0.0], [0.0]])
        for i in range(0, N):
            self.x_hat = self.x_hat + self.Ts/float(N)*(
                self.A*(self.x_hat - xe)
                + self.B*(self.F_d1 - P.Fe)
                + self.L*(y-self.C*self.x_hat)
            )

    def updateForce(self, Th, Ph):
        self.Th_d1 = Th
        self.Ph_d1 = Ph

    def integrateError(self, error_x, error_y):
        self.integrator_x = self.integrator_x + (self.Ts/2.0)*(error + self.error_x_d1)
        self.integrator_y = self.integrator_y + (self.Ts/2.0)*(error + self.error_y_d1)
        self.error_x_d1 = error_x
        self.error_y_d1 = error_y

    def integratorAntiWindup(self, F, F_unsat):
        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator + P.Ts/self.ki*(F-F_unsat)

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
