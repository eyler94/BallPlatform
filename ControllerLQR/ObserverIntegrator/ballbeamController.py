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
        self.Kx = P13.Kx               # state feedback gain
        self.kxi = P13.kxi             # Integral gain
        self.Ky = P13.Ky               # state feedback gain
        self.kyi = P13.kyi             # Integral gain
        self.L = P13.L                 # observer gain
        self.A = P13.A                 # system model
        self.B = P13.B
        self.C = P13.C
        self.theta_limit = P.Thmax          # Maximum force
        self.phi_limit = P.Phmax
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, x_ref, y_ref, c_state):
        # y_r is the referenced input
        # y is the current state
        x_r = x_ref
        x = c_state[0]
        y_r = y_ref
        y = c_state[1]

        # update the observer and extract z_hat
        self.updateObserver(c_state)
        x_hat = self.z_hat[0]
        y_hat = self.z_hat[1]

        # integrate error
        error_x = x_r - x
        error_y = y_r - y
        self.integrateError(error_x, error_y)

        # Construct the state
        xe = np.matrix([[0.0], [0.0]]) ### Change in real life to use beginning position of the ball.
        ye = np.matrix([[0.0], [0.0]]) ### Change in real life to use beginning position of the ball.
        x_tilde = np.matrix([[self.z_hat.item(0)],[self.z_hat.item(1)]]) - xe
        y_tilde = np.matrix([[self.z_hat.item(2)],[self.z_hat.item(3)]]) - ye

        # Compute the state feedback controller
        Theta_unsat = float(-self.Kx*x_tilde) - self.kxi*self.integrator_x
        Phi_unsat = float(-self.Ky*y_tilde) - self.kyi*self.integrator_y
        Theta = self.saturate(Theta_unsat,self.theta_limit)
        Phi = self.saturate(Phi_unsat,self.phi_limit)
        self.integrator_x_AntiWindup(Theta, Theta_unsat)
        self.integrator_y_AntiWindup(Phi, Phi_unsat)
        self.updateAngle(Theta, Phi)
        return [Theta, Phi]

    def updateObserver(self, z_m):
        N = 10
        z = np.matrix([
            [z_m[0]],
            [z_m[1]]])
        ze = np.matrix([[0.0], [0.0], [0.0], [0.0]]) ### Change in real life to use beginning position of the ball.
        for i in range(0, N):
            self.z_hat = self.z_hat + self.Ts/float(N)*(
                self.A*(self.z_hat - ze)
                + self.B*np.matrix([[self.Th_d1],[self.Ph_d1]])
                + self.L*(z-self.C*self.z_hat))

    def updateAngle(self, Theta, Phi):
        self.Th_d1 = Theta
        self.Ph_d1 = Phi

    def integrateError(self, error_x, error_y):
        self.integrator_x = self.integrator_x + (self.Ts/2.0)*(error_x + self.error_x_d1)
        self.integrator_y = self.integrator_y + (self.Ts/2.0)*(error_y + self.error_y_d1)
        self.error_x_d1 = error_x
        self.error_y_d1 = error_y

    def integrator_x_AntiWindup(self, Theta, Theta_unsat):
        # integrator anti - windup
        if self.kxi != 0.0:
            self.integrator_x = self.integrator_x + P.Ts/self.kxi*(Theta-Theta_unsat)

    def integrator_y_AntiWindup(self, Phi, Phi_unsat):
        # integrator anti - windup
        if self.kyi != 0.0:
            self.integrator_y = self.integrator_y + P.Ts/self.kyi*(Phi-Phi_unsat)

    def saturate(self,u,limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u
