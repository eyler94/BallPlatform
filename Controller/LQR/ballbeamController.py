import numpy as np
import ballbeamParam as P
import scipy.linalg

import pandas as pd
pd.set_option('display.width', 320)
pd.set_option('display.max_columns', 12)
np.set_printoptions(linewidth=320)

def lqr(A,B,Q,R):
    X = np.matrix(scipy.linalg.solve_continuous_are(A,B,Q,R))
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
    #eigenVals, eigenVect = scipy.linalg.eig(A-B*K)
    return K

class ballbeamController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self,x0,y0):
        A = np.array([[0., 1., 0., 0.],
                      [0., 0., 0., 0.],
                      [0., 0., 0., 1.],
                      [0., 0., 0., 0]])
        g = 9.81
        B = np.array([[0., 0.],
                      [-g, 0.],
                      [0., 0.],
                      [0., -g]])
        C = np.array([[1., 0., 0., 0.],
                      [0., 0., 1., 0.]])

        Caug = np.hstack((C,np.zeros((2,2))))
        Atemp = np.hstack((A,np.zeros((4,2))))
        Aaug = np.vstack((Atemp,-Caug))
        Baug = np.vstack((B,np.zeros((2,2))))

        xmax = 0.1016
        xdotmax = 0.1016
        xIntmax = 1.
        ymax = 0.1016
        ydotmax = 0.1016
        yIntmax = 1.
        Q = np.diagflat([[1./xmax**2, 1./xdotmax**2],[1./ymax**2, 1./ydotmax**2],[1./xIntmax**2, 1./yIntmax**2]])

        R = np.eye(2)
        max_u = np.pi/24.
        R[0,0] = 1./(max_u)**2
        R[1,1] = 1./(max_u)**2

        Kall = np.asarray(lqr(Aaug,Baug,Q,R))
        self.K = Kall[:,:4]#np.array([[-1.342, -0.5392, 0., 0.],[0., 0., -1.342, -0.5392]])
        self.Ki = Kall[:,4:]
        self.Kr = -np.linalg.pinv(Caug@np.linalg.pinv(Aaug-Baug@Kall)@Baug)

        self.x = x0
        self.x_d1 = x0
        self.x_dot = 0.

        self.y = y0
        self.y_d1 = y0
        self.y_dot = 0.

        self.state_hat = np.array([[self.x, self.x_dot, self.y, self.y_dot]]).T
        self.limit = np.pi/6

        ##integrator stuff
        self.Th_d1 = 0.0  # Computed Angle, delayed by one sample
        self.Ph_d1 = 0.0
        self.integrator_x = 0.0        # integrator
        self.integrator_y = 0.0        # integrator
        self.error_x_d1 = 0.0          # error signal delayed by 1 sample
        self.error_y_d1 = 0.0          # error signal delayed by 1 sample
        self.kxi = 0.#5            # Integral gain
        self.kyi = 0.#5             # Integral gain

        ##Derivative stuff
        length=10
        self.xArray = np.ones((1,length))
        self.yArray = np.ones((1,length))
        self.first = True

    def u(self, state_r, state_c):
        self.x = state_c[0]
        self.y = state_c[1]
        self.derivatives()

        #integrate error
        error_x = state_r[0] - self.x
        error_y = state_r[1] - self.y
        self.integrateError(error_x, error_y)

        state_hat = np.array([[self.x, self.x_dot, self.y, self.y_dot]]).T
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

    def derivatives(self):
        if self.first:
            self.xArray = self.xArray*self.x
            self.yArray = self.yArray*self.y
            self.first = False
        else:
            for i in range(9,0,-1):
                self.xArray[0][i]=self.xArray[0][i-1]
                self.yArray[0][i]=self.yArray[0][i-1]
            self.xArray[0][0]=self.x
            self.yArray[0][0]=self.y
        self.x = np.average(self.xArray)
        self.y = np.average(self.yArray)

        self.x_dot = (self.x-self.x_d1)/P.Ts
        self.x_d1 = self.x
        self.y_dot = (self.y-self.y_d1)/P.Ts
        self.y_d1 = self.y

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
