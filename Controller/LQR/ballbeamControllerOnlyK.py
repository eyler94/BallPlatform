import numpy as np
import ballbeamParam as P
import scipy.linalg

printWide = True
if printWide:
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

        ###Determine K and Ki
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

        distMax = 0.00051016
        dotMax = 1.
        IntMax = 0.5
        xmax = distMax
        xdotmax = dotMax
        xIntmax = IntMax
        ymax = distMax
        ydotmax = dotMax
        yIntmax = IntMax
        Q = np.diagflat([[1./xmax**2, 1./xdotmax**2],[1./ymax**2, 1./ydotmax**2],[1./xIntmax**2, 1./yIntmax**2]])

        R = np.eye(2)
        max_u = 0.05
        R[0,0] = 1./(max_u)**2
        R[1,1] = 1./(max_u)**2

        Kall = np.asarray(lqr(Aaug,Baug,Q,R))
        self.K = Kall[:,:4]#np.array([[-1.342, -0.5392, 0., 0.],[0., 0., -1.342, -0.5392]])
        self.Kr = -np.linalg.pinv(Caug@np.linalg.pinv(Aaug-Baug@Kall)@Baug)

        self.state = np.array([[x0, 0., y0, 0.]]).T
        self.state_d1 = np.array([[x0, 0., y0, 0.]]).T
        self.first = True
        self.length=2
        self.state_Array = np.ones((2,self.length))

        ### Integration stuff
        self.Ki = Kall[:,4:]
        self.error_state = np.zeros((2,1))
        self.int_error = np.zeros((2,1))
        self.error_d1 = np.zeros((2,1))

        ### Saturation protection
        self.Ts = P.Ts
        self.limit = np.pi

    def u(self, req, cur):
        self.state[0,0]=cur[0,0]
        self.state[2,0]=cur[1,0]
        self.derivatives()

        self.error_state = req-cur

        deriv_lim = 0.2
        if self.state[1,0] <= deriv_lim:
            self.integrateError(self.error_state, 0)
        if self.state[3,0] <= deriv_lim:
            self.integrateError(self.error_state, 1)
        # print("error:",self.error_state,"\nIntEr:",self.int_error)

        # u_unsat = self.Kr@req-self.K@self.state-self.Ki@self.int_error
        u_unsat = -req-self.K@self.state-self.Ki@self.int_error

        u_sat = self.saturate(u_unsat)

        # self.integratorAntiWindup(u_sat,u_unsat)

        # u = np.ones((2,1))*0.000000001
        return u_sat

    def derivatives(self):
        # print("state",self.state,"\nprev state:",self.state_d1)
        if self.first:
            self.state_Array[0,:] = self.state_Array[0,:]*self.state[0,0]
            self.state_Array[1,:] = self.state_Array[1,:]*self.state[2,0]
            self.first = False
        else:
            for i in range(self.length-1,0,-1):
                self.state_Array[0][i]=self.state_Array[0][i-1]
                self.state_Array[1][i]=self.state_Array[1][i-1]
            self.state_Array[0][0]=self.state[0,0]
            self.state_Array[1][0]=self.state[2,0]
        self.state[0,0] = np.average(self.state_Array[0,:])
        self.state[2,0] = np.average(self.state_Array[1,:])

        self.state[1,0] = (self.state[0,0]-self.state_d1[0,0])/self.Ts
        self.state_d1[0,0] = self.state[0,0]
        self.state[3,0] = (self.state[2,0]-self.state_d1[2,0])/self.Ts
        self.state_d1[2,0] = self.state[2,0]
        # print("state",self.state,"\nprev state:",self.state_d1,"\nstate Array:",self.state_Array)

    def integrateError(self, error,spot):
        self.int_error[spot][0] = self.int_error[spot][0] + (self.Ts/2.)*(error[spot][0] + self.error_d1[spot][0])
        self.error_d1[spot][0] = error[spot][0]

    def saturate(self, u_unsat):
        u_sat = np.zeros((2,1))
        for spot in range(0,2):
            if u_unsat.item(spot) >= self.limit:
                u_sat[spot][0] = self.limit
            elif u_unsat.item(spot) <= -self.limit:
                u_sat[spot][0] = -self.limit
            else:
                u_sat[spot][0] = u_unsat.item(spot)
        return u_sat

    def integratorAntiWindup(self, u_sat, u_unsat):
        self.int_error = self.int_error + self.Ts/self.Ki*(u_sat-u_unsat)
