"""
(C) Copyright 2015 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)

Version History:
- June 4, 2015 - Murilo M. Marinho
-- Added license headers.
-- Changed print statements to be compatible with python 3.5
- May 28, 2015 - Murilo M. Marinho
-- Initial Version
"""

import numpy as np
from scipy import linalg as LA
import unittest
from DQ import *
from math import cos,sin

class DQ_kinematics:


    def __init__(self,dh_matrix,convention):
        
        if str(convention) != "standard" and str(convention) != "modified":
            raise RuntimeError("Bad DQ_kinematics(dh_matrix, convention) call: convention must be 'standard' or 'modified' ")
        
        (rows,cols) = dh_matrix.shape        
        if rows != 4 and rows != 5:
            raise RuntimeError("Bad DQ_kinematics(dh_matrix, convention) call: dh_matrix should be 5xn or 4xn")
        
        self.dh_matrix            = dh_matrix;
        self.curr_base            = DQ(1);
        self.curr_effector        = DQ(1);
        self.dh_matrix_convention = convention;

    def links(self):
        (rows,cols) = self.dh_matrix.shape
        return int(cols) 

    def theta(self):
        return np.array(self.dh_matrix[0,:])
        
    def d(self):
        return np.array(self.dh_matrix[1,:])

    def a(self):
        return np.array(self.dh_matrix[2,:])

    def alpha(self):
        return np.array(self.dh_matrix[3,:])

    def dummy(self):
        (rows,cols) = self.dh_matrix.shape
        if rows > 4:
            return np.array(self.dh_matrix[4,:])
        else:
            return np.zeros(cols)

    def set_dummy(self,dummy_vector):
        (rows,cols) = self.dh_matrix.shape
        if dummy_vector.shape != (cols,):
            raise RuntimeError("Cannot change dummy status due to oddly shapped argument.")
        if rows > 4:
            self.dh_matrix[4,:] = dummy_vector
        else:
            print("No dummy information to change.")

    def n_dummy(self):
        return int(np.count_nonzero(self.dummy()))  

    def convention(self):
        return str(self.dh_matrix_convention)

    def base(self):
        return DQ(self.curr_base)

    def effector(self):
        return DQ(self.effector)

    def set_base(self,base):
        self.curr_base = base
        return DQ(self.curr_base)

    def set_effector(self,effector):
        self.curr_effector = effector
        return DQ(self.curr_effector)

    def dh2dq(self,theta_ang,link_i):
        q = np.empty(8);

        theta = (self.theta())[link_i-1]
        d     = (self.d())[link_i-1];
        a     = (self.a())[link_i-1];
        alpha = (self.alpha())[link_i-1];

        if self.convention == str("standard"):

            q[0]=cos((theta_ang +  theta)/2.0)*cos(alpha/2.0);
            q[1]=cos((theta_ang +  theta)/2.0)*sin(alpha/2.0);
            q[2]=sin((theta_ang +  theta)/2.0)*sin(alpha/2.0);
            q[3]=sin((theta_ang +  theta)/2.0)*cos(alpha/2.0);
            d2=d/2.0;
            a2=a/2.0;
            q[4]= -d2*q[3] - a2*q[1];
            q[5]= -d2*q[2] + a2*q[0];
            q[6]=  d2*q[1] + a2*q[3];
            q[7]=  d2*q[0] - a2*q[2];
        else:

            h1 = cos((theta_ang +  theta )/2.0)*cos(alpha/2.0);
            h2 = cos((theta_ang +  theta )/2.0)*sin(alpha/2.0);
            h3 = sin((theta_ang +  theta )/2.0)*sin(alpha/2.0);
            h4 = sin((theta_ang +  theta )/2.0)*cos(alpha/2.0);
            q[0]= h1;
            q[1]= h2;
            q[2]= -h3;
            q[3]= h4;
            d2=d/2.0;
            a2=a/2.0;
            q[4]=-d2*h4 - a2*h2;
            q[5]=-d2*h3 + a2*h1;
            q[6]=-(d2*h2 + a2*h4);
            q[7]=d2*h1 - a2*h3;
        
        return q;

    def raw_fkm(self,theta):
        (size,) = theta.shape
        if size != (self.links() - self.n_dummy()):
            raise RuntimeError("Bad raw_fkm(theta_vec) call: Incorrect number of joint variables")

        q = DQ(1)
        j = 0;
        dummy = self.dummy()
        
        for i in range(0,self.links()):
            if dummy[i] == 1:
                q = q * self.dh2dq(0.0, i+1)
                j = j + 1
            else:
                q = q * self.dh2dq(theta[i-j], i+1)
        
        return q;

    def get_z(self,q):
    
        z = np.empty(8);

        z[0] = 0.0
        z[1]= q[1]*q[3] + q[0]*q[2]
        z[2]= q[2]*q[3] - q[0]* q[1]
        z[3]= (q[3]*q[3]-q[2]*q[2]-q[1]*q[1]+q[0]*q[0])/2.0
        z[4]= 0.0
        z[5]= q[1]*q[7]+q[5]*q[3]+q[0]*q[6]+q[4]*q[2]
        z[6]= q[2]*q[7]+q[6]*q[3]-q[0]*q[5]-q[4]*q[1]
        z[7]= q[3]*q[7]-q[2]*q[6]-q[1]*q[5]+q[0]*q[4]

        return z;

    def fkm(self,theta):
        q = self.curr_base * ( self.raw_fkm(theta) ) * self.curr_effector;
        return q;
  
    #def fkm(self,theta,ith):
    #    q = self.curr_base * ( self.raw_fkm(theta, ith) ) * curr_effector;
    #    return q;

    def jacobian(self,theta):

        q_effector = self.raw_fkm(theta);

        z = DQ(0)
        q = DQ(1)

        J = np.zeros((8,(self.links() - self.n_dummy())))

        ith = -1;
        for i in range(0,self.links()):
            # Use the standard DH convention
            if self.convention() == "standard" :
                z = self.get_z(q.q)
            # Use the modified DH convention
            else:
                alpha = (self.alpha())[i]
                a     = (self.a())[i]
                dummy = (self.dummy())[i]
                w     = DQ([0, 0, -sin( alpha ), cos( alpha ), 0, 0, -a*cos(alpha), -a*sin(alpha)])
                z     = 0.5 * q * w * q.conj()
                

                if dummy == 0:
                    q     = q * self.dh2dq(theta[ith+1], i+1)
                    aux_j = z * q_effector
                    for k in range(0,8):
                        J[k,ith+1] = aux_j.q[k]
                    ith = ith+1
                else:
                    #Dummy joints don't contribute to the Jacobian
                    q = q * self.dh2dq(0.0,(i+1))
        # Takes the base's displacement into account
        aux_J = haminus8(self.curr_effector)
        aux_J = np.dot(hamiplus8(self.curr_base),aux_J)
        J = np.dot(aux_J,J)

        return J


    #Alias
    analyticalJacobian = jacobian
    
DQ_kinematics.C8 = np.diag([1,-1,-1,-1,1,-1,-1,-1])
DQ_kinematics.C4 = np.diag([1,-1,-1,-1])    

def links(dqkin):
    return dqkin.links()
def theta(dqkin):
    return dqkin.theta()
def d(dqkin):
    return dqkin.d()
def a(dqkin):
    return dqkin.a()
def alpha(dqkin):
    return dqkin.alpha()
def dummy(dqkin):
    return dqkin.dummy()
def n_dummy(dqkin):
    return dqkin.n_dummy()
def convention(dqkin):
    return dqkin.convention()
def base(dqkin):
    return dqkin.base()
def effector(dqkin):
    return dqkin.effector()
def set_base(dqkin,dq):
    return dqkin.set_base(dq)
def set_effector(dqkin,dq):
    return dqkin.set_effector(dq)

def dh2dq(dqkin,theta_ang,i):
    return dqkin.dh2dq(theta_ang,i)
def get_z(dqkin,q):
    return dqkin.get_z(q)
def fkm(dqkin,theta):
    return dqkin.fkm(theta)
def jacobian(dqkin,theta):
    return dqkin.jacobian(theta)
analyticalJacobian = jacobian
def rotationJacobian(analytical_jacobian):
    return np.array(analytical_jacobian[0:4])
def jacobp(analytical_jacobian,x):
    dq_x = DQ(x);
    dq_x_conj_P = dq_x.P();
    dq_x_conj_P = dq_x_conj_P.conj();
    (rows,cols) = analytical_jacobian.shape
    aux_J1 = np.zeros((4,cols))
    aux_J2 = np.zeros((4,cols))
    for i in range(0,4):
        for j in range(0,cols):
            aux_J1[i,j] = analytical_jacobian[i,j]
            aux_J2[i,j] = analytical_jacobian[(i+4),j]

    aux = np.dot(hamiplus4(dq_x.D()),DQ_kinematics.C4);
    Jp = 2*(np.dot(haminus4(dq_x_conj_P),aux_J2)) + 2*(np.dot(aux,aux_J1));
    return Jp;

if __name__ == '__main__':

    dq_kinematics = DQ_kinematics( np.array(np.mat('1,1;2,2;3,3;4,4;0,0')) ,"standard") 

    print(dq_kinematics.theta())
    print(dq_kinematics.dummy())
    print(dq_kinematics.n_dummy())
    print(dq_kinematics.raw_fkm(np.array([0,0])))
    print(dq_kinematics.jacobian(np.array([0,0])))
    print(dq_kinematics.fkm(np.array([0,0])))
    print(rotationJacobian(dq_kinematics.jacobian(np.array([0,0]))))
    print(jacobp(dq_kinematics.jacobian(np.array([0,0])),[1,2,3,4,5,6,7,8]))




