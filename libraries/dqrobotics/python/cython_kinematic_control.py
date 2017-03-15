from DQ import *
from DQ_kinematics import *
import numpy as np
import scipy.linalg as LA
from math import pi
from DQ_CYTHON import DQ_CYTHON

#Create a new DQ_kinematics object with the A2ARM standard Denavit-Hartenberg parameters
cython_kine = DQ_CYTHON();

# Basic definitions for the simulation
theta  = np.ones(7)*0.5
thetad = np.array([0.5,0.5,0.5,0.5,0.5,0.5,0.3])

# Desired end-effector pose
xd = cython_kine.fkm(thetad);

error = 1;
epsilon = 0.01;
K = 0.5;

while LA.norm(error) > epsilon:
    x = cython_kine.fkm(theta);
    J = cython_kine.jacobian(theta);
    error = vec8(xd-x);
    dtheta = np.dot(np.dot(LA.pinv(J),K),error);
    theta = theta + dtheta;
    print LA.norm(dtheta)
    # print LA.norm(error)
    # print theta
    # print "Error"
    # print x
    # print xd
    # print error
