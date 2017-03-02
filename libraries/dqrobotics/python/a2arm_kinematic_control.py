from DQ import *
from DQ_kinematics import *
import numpy as np
import scipy.linalg as LA
from math import pi
from DQ_A2ARM import DQ_A2ARM

#Create a new DQ_kinematics object with the A2ARM standard Denavit-Hartenberg parameters
a2arm_kine = DQ_A2ARM();

# Basic definitions for the simulation
theta  = np.array([0,0,-pi/2,0,pi/2,0,0])
thetad = np.array([0.1,0,-pi/2,0,pi/2,0,0])

# Desired end-effector pose
xd = a2arm_kine.fkm(thetad);

error = 1;
epsilon = 0.001;
K = 0.5;

while LA.norm(error) > epsilon:
    x = a2arm_kine.fkm(theta);
    J = a2arm_kine.jacobian(theta);
    error = vec8(xd-x);
    theta = theta + np.dot(np.dot(LA.pinv(J),K),error);
    print LA.norm(error)
    print theta
