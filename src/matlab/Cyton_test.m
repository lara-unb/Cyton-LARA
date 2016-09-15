%% Script to plot Arduino
% @author Rafael

close all
clear all
clc

% Perform the connection
delete(instrfind({'Port'},{'/dev/ttyUSB0'}));
serialPort=serial('/dev/ttyUSB0');
serialPort.BaudRate=9600;
fopen(serialPort);

desired_theta = [0,0,0,0,0,0,0]

fgetl(serialPort);

x = desired_theta

while (size(x,1)< 100)% (~isempty(serialPort))
    str = fgetl(serialPort);
    str = strtrim(str);
    str = strsplit(str, ' ');
    for n = 1:7
        if n == 2
            desired_theta(n) = -str2double(str{n});
        else
            desired_theta(n) = str2double(str{n});
        end
    end
    x = [x;deg2rad(desired_theta)];
end

plot(x)
CytonSimulation(x(4,:))

fclose(serialPort); 
delete(serialPort);

plot(x)