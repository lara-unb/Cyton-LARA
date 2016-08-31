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
from math import sqrt,acos,sin,cos

class DQ:

    THRESHOLD = 1e-12

    def _apply_treshold_(dq):
        for i in range(0,8):
            if abs(dq.q[i]) < DQ.THRESHOLD:
                dq.q[i] = 0

    def __init__(self,q):
        
        if isinstance(q,DQ):
            self.q = np.array(q.q,np.float64)
            return
        else:
            a = np.array(q,np.float64)
            self.q = np.array([0,0,0,0,0,0,0,0],np.float64)

        # Scalar
        if np.ndim(a)==0:
            self.q[0]=q
        # Array
        elif np.ndim(a)==1:
            if len(a) > 8:
                raise RuntimeError("Tried to initialize DQ with size > 8")
            else:
                for i in range(0,len(a)):
                    self.q[i]=a[i]
        # Matrix
        else:
            raise RuntimeError("DQ initialization can only accept array or scalar.")

        DQ._apply_treshold_(self)

    ## Binary Operators
    def __eq__(self,other):
        return (self.q==DQ(other).q).all()
    
    def __ne__(self,other):
        if (self.q==DQ(other).q).all():
            return False
        else:
            return True

    def __add__(self,other):
        dq  = DQ(0.0)
        dq1 = DQ(self)
        dq2 = DQ(other)        

        for n in range(0,8):
            dq.q[n] = dq1.q[n] + dq2.q[n]

        DQ._apply_treshold_(dq)
        return dq

    def __sub__(self,other):
        dq  = DQ(0.0)
        dq1 = DQ(self)
        dq2 = DQ(other)        

        for n in range(0,8):
            dq.q[n] = dq1.q[n] - dq2.q[n]

        DQ._apply_treshold_(dq)
        return dq

    def __rsub__(self,other):
        dq  = DQ(0.0)
        dq1 = DQ(other)
        dq2 = DQ(self)        

        for n in range(0,8):
            dq.q[n] = dq1.q[n] - dq2.q[n]

        DQ._apply_treshold_(dq)
        return dq

    def __mul__(self,other):
        dq = DQ(0)

        dq1 = DQ(self )
        dq2 = DQ(other)

        dq1d = dq1.D()
        dq2p = dq2.P()
        dq2d = dq2.D()

        dq.q[0] = dq1.q[0]*dq2.q[0] - dq1.q[1]*dq2.q[1] - dq1.q[2]*dq2.q[2] - dq1.q[3]*dq2.q[3];
        dq.q[1] = dq1.q[0]*dq2.q[1] + dq1.q[1]*dq2.q[0] + dq1.q[2]*dq2.q[3] - dq1.q[3]*dq2.q[2];
        dq.q[2] = dq1.q[0]*dq2.q[2] - dq1.q[1]*dq2.q[3] + dq1.q[2]*dq2.q[0] + dq1.q[3]*dq2.q[1];
        dq.q[3] = dq1.q[0]*dq2.q[3] + dq1.q[1]*dq2.q[2] - dq1.q[2]*dq2.q[1] + dq1.q[3]*dq2.q[0];

        dq.q[4] = dq1.q[0]*dq2d.q[0] - dq1.q[1]*dq2d.q[1] - dq1.q[2]*dq2d.q[2] - dq1.q[3]*dq2d.q[3];
        dq.q[5] = dq1.q[0]*dq2d.q[1] + dq1.q[1]*dq2d.q[0] + dq1.q[2]*dq2d.q[3] - dq1.q[3]*dq2d.q[2];
        dq.q[6] = dq1.q[0]*dq2d.q[2] - dq1.q[1]*dq2d.q[3] + dq1.q[2]*dq2d.q[0] + dq1.q[3]*dq2d.q[1];
        dq.q[7] = dq1.q[0]*dq2d.q[3] + dq1.q[1]*dq2d.q[2] - dq1.q[2]*dq2d.q[1] + dq1.q[3]*dq2d.q[0];

        dq.q[4] = dq.q[4] + dq1d.q[0]*dq2p.q[0] - dq1d.q[1]*dq2p.q[1] - dq1d.q[2]*dq2p.q[2] - dq1d.q[3]*dq2p.q[3];
        dq.q[5] = dq.q[5] + dq1d.q[0]*dq2p.q[1] + dq1d.q[1]*dq2p.q[0] + dq1d.q[2]*dq2p.q[3] - dq1d.q[3]*dq2p.q[2];
        dq.q[6] = dq.q[6] + dq1d.q[0]*dq2p.q[2] - dq1d.q[1]*dq2p.q[3] + dq1d.q[2]*dq2p.q[0] + dq1d.q[3]*dq2p.q[1];
        dq.q[7] = dq.q[7] + dq1d.q[0]*dq2p.q[3] + dq1d.q[1]*dq2p.q[2] - dq1d.q[2]*dq2p.q[1] + dq1d.q[3]*dq2p.q[0];

        DQ._apply_treshold_(dq)
        return dq

    def __rmul__(self,other):
        dq = DQ(0)

        dq1 = DQ(other )
        dq2 = DQ(self)

        dq1d = dq1.D()
        dq2p = dq2.P()
        dq2d = dq2.D()

        dq.q[0] = dq1.q[0]*dq2.q[0] - dq1.q[1]*dq2.q[1] - dq1.q[2]*dq2.q[2] - dq1.q[3]*dq2.q[3];
        dq.q[1] = dq1.q[0]*dq2.q[1] + dq1.q[1]*dq2.q[0] + dq1.q[2]*dq2.q[3] - dq1.q[3]*dq2.q[2];
        dq.q[2] = dq1.q[0]*dq2.q[2] - dq1.q[1]*dq2.q[3] + dq1.q[2]*dq2.q[0] + dq1.q[3]*dq2.q[1];
        dq.q[3] = dq1.q[0]*dq2.q[3] + dq1.q[1]*dq2.q[2] - dq1.q[2]*dq2.q[1] + dq1.q[3]*dq2.q[0];

        dq.q[4] = dq1.q[0]*dq2d.q[0] - dq1.q[1]*dq2d.q[1] - dq1.q[2]*dq2d.q[2] - dq1.q[3]*dq2d.q[3];
        dq.q[5] = dq1.q[0]*dq2d.q[1] + dq1.q[1]*dq2d.q[0] + dq1.q[2]*dq2d.q[3] - dq1.q[3]*dq2d.q[2];
        dq.q[6] = dq1.q[0]*dq2d.q[2] - dq1.q[1]*dq2d.q[3] + dq1.q[2]*dq2d.q[0] + dq1.q[3]*dq2d.q[1];
        dq.q[7] = dq1.q[0]*dq2d.q[3] + dq1.q[1]*dq2d.q[2] - dq1.q[2]*dq2d.q[1] + dq1.q[3]*dq2d.q[0];

        dq.q[4] = dq.q[4] + dq1d.q[0]*dq2p.q[0] - dq1d.q[1]*dq2p.q[1] - dq1d.q[2]*dq2p.q[2] - dq1d.q[3]*dq2p.q[3];
        dq.q[5] = dq.q[5] + dq1d.q[0]*dq2p.q[1] + dq1d.q[1]*dq2p.q[0] + dq1d.q[2]*dq2p.q[3] - dq1d.q[3]*dq2p.q[2];
        dq.q[6] = dq.q[6] + dq1d.q[0]*dq2p.q[2] - dq1d.q[1]*dq2p.q[3] + dq1d.q[2]*dq2p.q[0] + dq1d.q[3]*dq2p.q[1];
        dq.q[7] = dq.q[7] + dq1d.q[0]*dq2p.q[3] + dq1d.q[1]*dq2p.q[2] - dq1d.q[2]*dq2p.q[1] + dq1d.q[3]*dq2p.q[0];

        DQ._apply_treshold_(dq)
        return dq

    ## Unary Operators
    def P(self):
        return DQ(self.q[0:4])

    def D(self):
        return DQ(self.q[4:8])

    def Re(self):
        return DQ([self.q[0],0,0,0,self.q[4],0,0,0])

    def Im(self):
        q = self.q
        return DQ([0,q[1],q[2],q[3],0,q[5],q[6],q[7]])

    def conj(self):
        q = self.q
        return DQ([q[0],-q[1],-q[2],-q[3],q[4],-q[5],-q[6],-q[7]]);
        
    def norm(self):
        norm = DQ(0)

        if self.P == 0:
            return norm
        else:
            norm = self.conj()*self
            norm.q[0] = sqrt(norm.q[0])
            norm.q[4] = norm.q[4]/[2*norm.q[0]]

            DQ._apply_treshold_(norm)

            return norm;

    def inv(self):
        aux  = self
        aux2 = DQ(0)

        #inverse calculation
        aux2 = aux * aux.conj(); #(dq norm)^2
        inv =  DQ([(1/aux2.q[0]),0,0,0,(-aux2.q[4]/(aux2.q[0]*aux2.q[0])),0,0,0])
        inv = (aux.conj() * inv)

        DQ._apply_treshold_(inv)
        return inv;  
    
    def translation(self):
        if self.norm() != 1:
            raise RuntimeError("Bad translation() call: not a unit dual quaternion")

        translation = DQ(self.P())
        translation = (2.0 * self.D() * translation.conj() )

        DQ._apply_treshold_(translation)
        return translation

    def rotation_axis(self):
        if self.norm() != 1:
            raise RuntimeError("Bad rotation_axis() call: not a unit dual quaternion")

        phi = acos(self.q[0]);
        if phi == 0:
            return DQ([0,0,0,1]) #This is only a convention;
        else:
            #rotation axis calculation
            rot_axis = self.P()
            rot_axis = ( rot_axis.Im() * (1/sin(phi)) )

            DQ._apply_treshold_(rotation_axis)
            return rot_axis

    def rotation_angle(self):
        if self.norm() != 1:
            raise RuntimeError("Bad rotation_angle() call: not a unit dual quaternion")

        #Rotation angle calculation
        rot_angle = 2*acos(self.q[0])

        return rot_angle

    def log(self):
        if self.norm() != 1:
            raise RuntimeError("Bad log() call: not a unit dual quaternion")

        # log calculation
        p = acos(self.q[0]) * self.rotation_axis(); #primary
        d = 0.5 * self.translation(); #dual
        log = DQ([p.q[0],p.q[1],p.q[2],p.q[3],d.q[0],d.q[1],d.q[2],d.q[3]]);

        DQ._apply_treshold_(log)    
        return log;       


    def exp(self):
        if self.Re() != 0.0:
            raise RuntimeError("Bad exp() call: Exponential operation is defined only for pure dual quaterions.")

        phi  = LA.norm((self.P()).q);

        if phi != 0.0:
           prim = cos(phi) + (sin(phi)/phi)*self.P()
        else:
           prim = DQ(1.0)

        exp = ( prim + DQ.E_*self.D()*prim );

        DQ._apply_treshold_(exp)    
        return exp;

    def T(self):
        if self.norm() != 1:
            raise RuntimeError("Bad T() call: not a unit dual quaternion")

        # tplus operator calculation
        tplus = self * (self.P()).conj();

        DQ._apply_treshold_(tplus)
        return tplus;

    def pinv(self):
        if self.norm() != 1:
            raise RuntimeError("Bad pinv() call: not a unit dual quaternion")

        # inverse calculation under decompositional multiplication
        tinv = self.conj();
        tinv = tinv.T() * self.T();
        pinv = tinv.conj() * self.conj();

        DQ._apply_treshold_(pinv)
        return pinv;


    def hamiplus4(self):
        op_hamiplus4 = np.zeros((4,4))
        op_hamiplus4[0,0] = self.q[0]; op_hamiplus4[0,1] = -self.q[1]; op_hamiplus4[0,2] = -self.q[2]; op_hamiplus4[0,3] = -self.q[3];
        op_hamiplus4[1,0] = self.q[1]; op_hamiplus4[1,1] =  self.q[0]; op_hamiplus4[1,2] = -self.q[3]; op_hamiplus4[1,3] =  self.q[2];
        op_hamiplus4[2,0] = self.q[2]; op_hamiplus4[2,1] =  self.q[3]; op_hamiplus4[2,2] =  self.q[0]; op_hamiplus4[2,3] = -self.q[1];
        op_hamiplus4[3,0] = self.q[3]; op_hamiplus4[3,1] = -self.q[2]; op_hamiplus4[3,2] =  self.q[1]; op_hamiplus4[3,3] =  self.q[0];

        return op_hamiplus4

    def haminus4(self):
        op_haminus4 = np.zeros((4,4))
        op_haminus4[0,0] = self.q[0]; op_haminus4[0,1] = -self.q[1]; op_haminus4[0,2] = -self.q[2]; op_haminus4[0,3] = -self.q[3];
        op_haminus4[1,0] = self.q[1]; op_haminus4[1,1] =  self.q[0]; op_haminus4[1,2] =  self.q[3]; op_haminus4[1,3] = -self.q[2];
        op_haminus4[2,0] = self.q[2]; op_haminus4[2,1] = -self.q[3]; op_haminus4[2,2] =  self.q[0]; op_haminus4[2,3] =  self.q[1];
        op_haminus4[3,0] = self.q[3]; op_haminus4[3,1] =  self.q[2]; op_haminus4[3,2] = -self.q[1]; op_haminus4[3,3] =  self.q[0];

        return op_haminus4

    def hamiplus8(self):
        op_hamiplus8 = np.zeros((8,8))
        op_hamiplus8[0,0] = self.q[0]; op_hamiplus8[0,1] = -self.q[1]; op_hamiplus8[0,2] = -self.q[2]; op_hamiplus8[0,3] = -self.q[3];
        op_hamiplus8[1,0] = self.q[1]; op_hamiplus8[1,1] =  self.q[0]; op_hamiplus8[1,2] = -self.q[3]; op_hamiplus8[1,3] =  self.q[2];
        op_hamiplus8[2,0] = self.q[2]; op_hamiplus8[2,1] =  self.q[3]; op_hamiplus8[2,2] =  self.q[0]; op_hamiplus8[2,3] = -self.q[1];
        op_hamiplus8[3,0] = self.q[3]; op_hamiplus8[3,1] = -self.q[2]; op_hamiplus8[3,2] =  self.q[1]; op_hamiplus8[3,3] =  self.q[0];

        op_hamiplus8[0,4] = 0; op_hamiplus8[0,5] = 0; op_hamiplus8[0,6] = 0; op_hamiplus8[0,7] = 0;
        op_hamiplus8[1,4] = 0; op_hamiplus8[1,5] = 0; op_hamiplus8[1,6] = 0; op_hamiplus8[1,7] = 0;
        op_hamiplus8[2,4] = 0; op_hamiplus8[2,5] = 0; op_hamiplus8[2,6] = 0; op_hamiplus8[2,7] = 0;
        op_hamiplus8[3,4] = 0; op_hamiplus8[3,5] = 0; op_hamiplus8[3,6] = 0; op_hamiplus8[3,7] = 0;

        op_hamiplus8[4,0] = self.q[4]; op_hamiplus8[4,1] = -self.q[5]; op_hamiplus8[4,2] = -self.q[6]; op_hamiplus8[4,3] = -self.q[7];
        op_hamiplus8[5,0] = self.q[5]; op_hamiplus8[5,1] =  self.q[4]; op_hamiplus8[5,2] = -self.q[7]; op_hamiplus8[5,3] =  self.q[6];
        op_hamiplus8[6,0] = self.q[6]; op_hamiplus8[6,1] =  self.q[7]; op_hamiplus8[6,2] =  self.q[4]; op_hamiplus8[6,3] = -self.q[5];
        op_hamiplus8[7,0] = self.q[7]; op_hamiplus8[7,1] = -self.q[6]; op_hamiplus8[7,2] =  self.q[5]; op_hamiplus8[7,3] =  self.q[4];

        op_hamiplus8[4,4] = self.q[0]; op_hamiplus8[4,5] = -self.q[1]; op_hamiplus8[4,6] = -self.q[2]; op_hamiplus8[4,7] = -self.q[3];
        op_hamiplus8[5,4] = self.q[1]; op_hamiplus8[5,5] =  self.q[0]; op_hamiplus8[5,6] = -self.q[3]; op_hamiplus8[5,7] =  self.q[2];
        op_hamiplus8[6,4] = self.q[2]; op_hamiplus8[6,5] =  self.q[3]; op_hamiplus8[6,6] =  self.q[0]; op_hamiplus8[6,7] = -self.q[1];
        op_hamiplus8[7,4] = self.q[3]; op_hamiplus8[7,5] = -self.q[2]; op_hamiplus8[7,6] =  self.q[1]; op_hamiplus8[7,7] =  self.q[0];

        return op_hamiplus8

    def haminus8(self):
        op_haminus8 = np.zeros((8,8))

        op_haminus8[0,0] = self.q[0]; op_haminus8[0,1] = -self.q[1]; op_haminus8[0,2] = -self.q[2]; op_haminus8[0,3] = -self.q[3];
        op_haminus8[1,0] = self.q[1]; op_haminus8[1,1] =  self.q[0]; op_haminus8[1,2] =  self.q[3]; op_haminus8[1,3] = -self.q[2];
        op_haminus8[2,0] = self.q[2]; op_haminus8[2,1] = -self.q[3]; op_haminus8[2,2] =  self.q[0]; op_haminus8[2,3] =  self.q[1];
        op_haminus8[3,0] = self.q[3]; op_haminus8[3,1] =  self.q[2]; op_haminus8[3,2] = -self.q[1]; op_haminus8[3,3] =  self.q[0];

        op_haminus8[0,4] = 0; op_haminus8[0,5] = 0; op_haminus8[0,6] = 0; op_haminus8[0,7] = 0;
        op_haminus8[1,4] = 0; op_haminus8[1,5] = 0; op_haminus8[1,6] = 0; op_haminus8[1,7] = 0;
        op_haminus8[2,4] = 0; op_haminus8[2,5] = 0; op_haminus8[2,6] = 0; op_haminus8[2,7] = 0;
        op_haminus8[3,4] = 0; op_haminus8[3,5] = 0; op_haminus8[3,6] = 0; op_haminus8[3,7] = 0;

        op_haminus8[4,0] = self.q[4]; op_haminus8[4,1] = -self.q[5]; op_haminus8[4,2] = -self.q[6]; op_haminus8[4,3] = -self.q[7];
        op_haminus8[5,0] = self.q[5]; op_haminus8[5,1] =  self.q[4]; op_haminus8[5,2] =  self.q[7]; op_haminus8[5,3] = -self.q[6];
        op_haminus8[6,0] = self.q[6]; op_haminus8[6,1] = -self.q[7]; op_haminus8[6,2] =  self.q[4]; op_haminus8[6,3] =  self.q[5];
        op_haminus8[7,0] = self.q[7]; op_haminus8[7,1] =  self.q[6]; op_haminus8[7,2] = -self.q[5]; op_haminus8[7,3] =  self.q[4];

        op_haminus8[4,4] = self.q[0]; op_haminus8[4,5] = -self.q[1]; op_haminus8[4,6] = -self.q[2]; op_haminus8[4,7] = -self.q[3];
        op_haminus8[5,4] = self.q[1]; op_haminus8[5,5] =  self.q[0]; op_haminus8[5,6] =  self.q[3]; op_haminus8[5,7] = -self.q[2];
        op_haminus8[6,4] = self.q[2]; op_haminus8[6,5] = -self.q[3]; op_haminus8[6,6] =  self.q[0]; op_haminus8[6,7] =  self.q[1];
        op_haminus8[7,4] = self.q[3]; op_haminus8[7,5] =  self.q[2]; op_haminus8[7,6] = -self.q[1]; op_haminus8[7,7] =  self.q[0];

        return op_haminus8

    def vec4(self):
        op_vec4 = np.zeros(4)
        for a in range(0,4):
            op_vec4[a]=self.q[a]
        return op_vec4

    def vec8(self):
        op_vec8 = np.zeros(8)
        for a in range(0,8):
            op_vec8[a]=self.q[a]
        return op_vec8

    def generalizedJacobian(self):
        jacobGen = np.zeros((8,8))
        
        jacobGen[0,0] = self.q[4]; jacobGen[0,1] =  self.q[5]; jacobGen[0,2] =  self.q[6]; jacobGen[0,3] =  self.q[7];
        jacobGen[1,0] = self.q[5]; jacobGen[1,1] = -self.q[4]; jacobGen[1,2] =  self.q[7]; jacobGen[1,3] = -self.q[6];
        jacobGen[2,0] = self.q[6]; jacobGen[2,1] = -self.q[7]; jacobGen[2,2] = -self.q[4]; jacobGen[2,3] =  self.q[5];
        jacobGen[3,0] = self.q[7]; jacobGen[3,1] =  self.q[6]; jacobGen[3,2] = -self.q[5]; jacobGen[3,3] = -self.q[4];

        jacobGen[0,4] =  self.q[0]; jacobGen[0,5] =  self.q[1]; jacobGen[0,6] =  self.q[2]; jacobGen[0,7] =  self.q[3];
        jacobGen[1,4] = -self.q[1]; jacobGen[1,5] =  self.q[0]; jacobGen[1,6] = -self.q[3]; jacobGen[1,7] =  self.q[2];
        jacobGen[2,4] = -self.q[2]; jacobGen[2,5] =  self.q[3]; jacobGen[2,6] =  self.q[0]; jacobGen[2,7] = -self.q[1];
        jacobGen[3,4] = -self.q[3]; jacobGen[3,5] = -self.q[2]; jacobGen[3,6] =  self.q[1]; jacobGen[3,7] =  self.q[0];

        jacobGen[4,0] =  self.q[0]; jacobGen[4,1] =  self.q[1]; jacobGen[4,2] =  self.q[2]; jacobGen[4,3] =  self.q[3];
        jacobGen[5,0] = -self.q[1]; jacobGen[5,1] =  self.q[0]; jacobGen[5,2] = -self.q[3]; jacobGen[5,3] =  self.q[2];
        jacobGen[6,0] = -self.q[2]; jacobGen[6,1] =  self.q[3]; jacobGen[6,2] =  self.q[0]; jacobGen[6,3] = -self.q[1];
        jacobGen[7,0] = -self.q[3]; jacobGen[7,1] = -self.q[2]; jacobGen[7,2] =  self.q[1]; jacobGen[7,3] =  self.q[0];

        jacobGen[4,4] = 0; jacobGen[4,5] = 0; jacobGen[4,6] = 0; jacobGen[4,7] = 0;
        jacobGen[5,4] = 0; jacobGen[5,5] = 0; jacobGen[5,6] = 0; jacobGen[5,7] = 0;
        jacobGen[6,4] = 0; jacobGen[6,5] = 0; jacobGen[6,6] = 0; jacobGen[6,7] = 0;
        jacobGen[7,4] = 0; jacobGen[7,5] = 0; jacobGen[7,6] = 0; jacobGen[7,7] = 0;

        return np.dot(2,jacobGen)

    def __str__(self):
        result = str()
        result+= str(self.q[0])+"  + "
        result+= str(self.q[1])+"i + "
        result+= str(self.q[2])+"j + "
        result+= str(self.q[3])+"k + e( "
        result+= str(self.q[4])+" + "
        result+= str(self.q[5])+"i + "
        result+= str(self.q[6])+"j + "
        result+= str(self.q[7])+"k )"
        return result

    
    # Alias for all functions
    __radd__ = __add__

    jacobG = generalizedJacobian

## DQ CLASS VARIABLES (THAT ARE DQ INSTANCES)
DQ.E_ = DQ([0,0,0,0,1,0,0,0])
DQ.i_ = DQ([0,1,0,0,0,0,0,0])
DQ.j_ = DQ([0,0,1,0,0,0,0,0])
DQ.k_ = DQ([0,0,0,1,0,0,0,0])

def P(dq):
    return dq.P()
def D(dq):
    return dq.D()
def Re(dq):
    return dq.Re()
def Im(dq):
    return dq.Im()
def conj(dq):
    return dq.conj()
def norm(dq):
    return dq.norm()
def inv(dq):
    return dq.inv()
def translation(dq):
    return dq.translation()
def rotation_axis(dq):
    return dq.rotation_axis()
def rotation_angle(dq):
    return dq.rotation_angle()
def log(dq):
    return dq.log()
def exp(dq):
    return dq.exp()
def T(dq):
    return dq.T()
def pinv(dq):
    return dq.pinv()
def dec_mult(dq1,dq2):
    return dq2.T()*dq1.T()*dq2.P()*dq1.P()
def hamiplus4(dq):
    return dq.hamiplus4()
def haminus4(dq):
    return dq.haminus4()
def hamiplus8(dq):
    return dq.hamiplus8()
def haminus8(dq):
    return dq.haminus8()
def vec4(dq):
    return dq.vec4()
def vec8(dq):
    return dq.vec8()
def generalizedJacobian(dq):
    return dq.generalizedJacobian()



class TestDQ(unittest.TestCase):
    
    def test_unary_operators(self):
        a = DQ([1,2,3,4,5,6,7,8])
        b = DQ([1,0,0,0,0,1,1,1])
        dummy_array = np.array([1,2,3,4,5,6,7,8],dtype=np.float64)
        self.assertTrue((a.q==dummy_array).all())

        print(DQ(0)==DQ([0]))
        self.assertEqual( DQ(0), DQ([0]) )
        
        print(a.P())
        self.assertEqual( a.P() , DQ([1,2,3,4]) )

        print(a.D())
        self.assertEqual( a.D() , DQ([5,6,7,8]) )

        print(a.Re())
        self.assertEqual( a.Re() , DQ([1,0,0,0,5,0,0,0]) )

        print(a.Im())
        self.assertEqual( a.Im() , DQ([0,2,3,4,0,6,7,8]) )

        print(a.conj())
        self.assertEqual( a.conj() , DQ([1,-2,-3,-4,5,-6,-7,-8]) )

        print(a.norm())

        print(a.inv())

        print(b.norm() != 1)
        print(DQ([1,0,0,0,0,1,1,1]).translation())
        print(DQ([1,0,0,0,0,1,1,1]).rotation_axis())
        print(DQ([1,0,0,0,0,1,1,1]).rotation_angle())
        print(DQ([1,0,0,0,0,1,1,1]).log())
        print((DQ([1,0,0,0,0,1,1,1]).log()).exp())
        print(DQ([1,0,0,0,0,1,1,1]).T())
        print(DQ([1,0,0,0,0,1,1,1]).pinv())
        print(DQ([1,0,0,0,0,1,1,1]).hamiplus4())
        print(DQ([1,0,0,0,0,1,1,1]).haminus4())
        print(DQ([1,0,0,0,0,1,1,1]).hamiplus8())
        print(DQ([1,0,0,0,0,1,1,1]).haminus8())
        print(DQ([1,0,0,0,0,1,1,1]).vec4())
        print(DQ([1,0,0,0,0,1,1,1]).vec8())
        print(DQ([1,0,0,0,0,1,1,1]).generalizedJacobian())
        print(DQ([1,0,0,0,0,1,1,1]).jacobG())
        print(dec_mult(b,b))

        #Operators
        self.assertEqual( a-1 , DQ([0,2,3,4,5,6,7,8]) )
        self.assertEqual( a+1 , DQ([2,2,3,4,5,6,7,8]) )
        self.assertEqual( 1+a , DQ([2,2,3,4,5,6,7,8]) )
        self.assertEqual( 1-a , DQ([0,-2,-3,-4,-5,-6,-7,-8]) )
        self.assertEqual( 1*a , DQ([1,2,3,4,5,6,7,8]) )
        self.assertEqual( a*1 , DQ([1,2,3,4,5,6,7,8]) )

        self.assertEqual( a.P(),P(a) )
        self.assertEqual( a.D(),D(a) )
        self.assertEqual( a.Re(),Re(a) )
        self.assertEqual( a.Im(),Im(a) )

if __name__ == '__main__':
    unittest.main()


    
