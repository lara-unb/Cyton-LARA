% cython = DQ_CYTHON returns a DQ_kinematics object using the modified
% Denavit-Hartenberg parameters of the CYTHON robotic arm

% (C) Copyright 2015 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br
%     Rafael Lima - raffaellimma@gmail.com

function cython = DQ_CYTHON
   %% Definitions for DQ_kinematics
    cython_DH_theta=  [0     , 0      , 0     , 0      , pi    , pi/2   , 0     ];
    cython_DH_d =     [0.0379, -0.0046, 0.145 , -0.011 , 0.175 , 0.0074 , 0.0   ];
    cython_DH_a =     [0.0   , 0.0    , 0.0   , 0.0    , 0.0   , 0.0    , 0.0677];
    cython_DH_alpha = [0     , pi/2   , -pi/2 , pi/2   , -pi/2 , pi/2   , pi/2  ];

    cython_DH_matrix = [cython_DH_theta;
        cython_DH_d;
        cython_DH_a;
        cython_DH_alpha;];

    cython = DQ_kinematics(cython_DH_matrix, 'modified');
end