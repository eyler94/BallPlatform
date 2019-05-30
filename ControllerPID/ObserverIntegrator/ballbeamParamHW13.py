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
# tuning parameters
tr_z = 2        # rise time for z
tr_theta = 0.625    # rise time for z_dot
zeta_z = 0.707  # damping ratio position
zeta_th = 0.707  # damping ratio angle
integrator_pole = -5.0
# pick observer poles
wn_th_obs = 5.0*2.2/tr_theta
wn_z_obs = 5.0*2.2/tr_z

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, 0.0, 0.0, 0.0]])

Ax = np.matrix([[0.0, 1.0],
                [0.0, 0.0]])

Ay = np.matrix([[0.0, 1.0],
                [0.0, 0.0]])

B = np.matrix([[0.0, 0.0],
               [-P.g, 0.0],
               [0.0, 0.0],
               [0.0, -P.g]])

Bx = np.matrix([[0.0],
               [-P.g]])

By = np.matrix([[0.0],
               [-P.g]])

C = np.matrix([[1.0, 0.0, 0.0, 0.0],
               # [0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0, 0.0]])#,
               # [0.0, 0.0, 0.0, 1.0]])
#
# Cx = np.matrix([[1.0, 0.0],
#                 [0.0, 1.0]])
#
# Cy = np.matrix([[1.0, 0.0],
#                 [0.0, 1.0]])

# form augmented system
Ax1 = np.matrix([[0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0],
               [-1.0, 0.0, 0.0]])

Ay1 = np.matrix([[0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0],
               [-1.0, 0.0, 0.0]])

Bx1 = np.matrix([[0.0],
               [-P.g],
               [0.0]])

By1 = np.matrix([[0.0],
               [-P.g],
               [0.0]])

# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly = np.convolve(
    np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                [1, 2*zeta_th*wn_th, wn_th**2]),
    np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(Ax1, Bx1)) !=3:
    print("The system is not controllable")
else:
    Kx1 = cnt.acker(Ax1, Bx1, des_poles)
    Kx = np.matrix([Kx1.item(0), Kx1.item(1)])#, K1.item(2), K1.item(3)])
    kxi = Kx1.item(2)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(Ay1, By1)) !=3:
    print("The system is not controllable")
else:
    Ky1 = cnt.acker(Ay1, By1, des_poles)
    Ky = np.matrix([Ky1.item(0), Ky1.item(1)])#, K1.item(2), K1.item(3)])
    kyi = Ky1.item(2)


# compute observer gains
des_obs_char_poly = np.convolve([1, 2*zeta_z*wn_z_obs, wn_z_obs**2],
                                [1, 2*zeta_th*wn_th_obs, wn_th_obs**2])
des_obs_poles = np.roots(des_obs_char_poly)

# Compute the gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 4:
    print("The system is not observable")
else:
    # place_poles returns an object with various properties.  The gains are accessed through .gain_matrix
    # .T transposes the matrix
    L = signal.place_poles(A.T, C.T, des_obs_poles).gain_matrix.T

print('K: ', Kx, Ky)
print('ki: ', kxi, kyi)
print('L^T: ', L.T)
