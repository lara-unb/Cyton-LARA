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
- March 11, 2016 - Rafael Lima
-- Modified for Cython Arm compatibility
"""

from DQ import *
from DQ_kinematics import *
import numpy as np
from math import pi

def DQ_CYTHON():

    cython_DH_theta = np.array([0     , 0      , 0     , 0      , pi    , pi/2   , 0     ])
    cython_DH_d     = np.array([0.0379, -0.0046, 0.145 , -0.011 , 0.175 , 0.0074 , 0.0   ])
    cython_DH_a     = np.array([0.0   , 0.0    , 0.0   , 0.0    , 0.0   , 0.0    , 0.0677])
    cython_DH_alpha = np.array([0     , pi/2   , -pi/2 , pi/2   , -pi/2 , pi/2   , pi/2  ])
    # comau_dummy =    np.array([0,0,0,0,0,0,0])

    cython_DH_matrix = np.array([cython_DH_theta,cython_DH_d,cython_DH_a,cython_DH_alpha])

    cython = DQ_kinematics(cython_DH_matrix,'modified')

    return cython