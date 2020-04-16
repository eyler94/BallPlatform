# ballbeam Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
import numpy as np
from scipy import signal
import control as cnt

####################################################
#                 State Space
####################################################
A = np.array([[0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, 0.0, 0.0, 0.0]])

B = np.array([[0.0, 0.0],
               [-P.g, 0.0],
               [0.0, 0.0],
               [0.0, -P.g]])

C = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0]])

Q = np.eye(4);
Q[0][0] = 1/(.1016**2);
Q[2][2] = 1/(.1016**2);

R = np.eye(2);
R[0][0] = 1/(np.pi/6)**2;
R[1][1] = 1/(np.pi/6)**2;

N=np.zeros([4,2])

[K, s, e] = cnt.lqr(A,B,Q,R,N)
