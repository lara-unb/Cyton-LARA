function [] = CytonKinematics(arduino, desired_theta)

%% Definitions for DQ_kinematics
% DH Parameters
cyton_DH_theta=  [0, 0, 0, 0, pi, pi/2, 0];
cyton_DH_d =     [0.0379, -0.0046, 0.145, -0.011, 0.175, 0.0074, 0.0];
cyton_DH_a =     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0677];
cyton_DH_alpha = [0, pi/2, -pi/2, pi/2, -pi/2, pi/2, pi/2];

cyton_DH_matrix = [cyton_DH_theta;
    cyton_DH_d;
    cyton_DH_a;
    cyton_DH_alpha;];

cyton_kine = DQ_kinematics(cyton_DH_matrix, 'modified');


%% Basic definitions for the simulation
initial_theta = [0,0,0,0,0,0,0]';

xd = cyton_kine.fkm(desired_theta);

error = 1;
epsilon = 0.001;
K = 0.6;
theta = initial_theta;

pause(1);
while norm(error) > epsilon  
    xm = cyton_kine.fkm(theta);
    J = cyton_kine.jacobian(theta);
    error = vec8(xd-xm);
    theta = theta + pinv(J)*K*error;
    fprintf(arduino, '%f %f %f %f %f %f %f ', rad2deg([theta(1) -theta(2) theta(3) theta(4) theta(5) theta(6) theta(7)]));
    disp (rad2deg([theta(1) -theta(2) theta(3) theta(4) theta(5) theta(6) theta(7)]));
    delay (0.8);    funciona

end
fprintf(arduino, '%f %f %f %f %f %f %f ', [999 0 0 0 0 0 0]);
