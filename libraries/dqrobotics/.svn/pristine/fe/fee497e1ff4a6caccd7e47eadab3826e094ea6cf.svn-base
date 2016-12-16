from DQ import *
from DQ_kinematics import *
import numpy as np
import scipy.linalg as LA
from math import pi
from DQ_COMAU import DQ_COMAU

#Create a new DQ_kinematics object with the COMAU SmartSiX modified Denavit-Hartenberg parameters           
comau_kine = DQ_COMAU();


# Basic definitions for the simulation
theta  = np.array([0,0,-pi/2,0,pi/2,0])
thetad = np.array([pi/2,0,-pi/2,0,pi/2,0])

# Desired end-effector pose
xd = comau_kine.fkm(thetad);

error = 1;
epsilon = 0.001;
K = 0.5;

while LA.norm(error) > epsilon:
    x = comau_kine.fkm(theta);
    J = comau_kine.jacobian(theta);
    error = vec8(xd-x);
    theta = theta + np.dot(np.dot(LA.pinv(J),K),error);
    print LA.norm(error)
