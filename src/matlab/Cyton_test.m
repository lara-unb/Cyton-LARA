%% Script to plot joint angles from Cyton
% @author Rafael
%

close all
clear all
clc

port = '/dev/ttyACM0';

% Perform the connection
delete(instrfind({'Port'},{port}));
serialPort=serial(port);
serialPort.BaudRate=9600;
fopen(serialPort);


% Init Variables
desired_theta = [0,0,0,0,0,0,0]
buffersize = 200;
x = zeros(buffersize,7);
i = 1;

% Remove First Line
fgetl(serialPort);

while (~isempty(serialPort))
    % Read Data
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
    
    % Save Data
    x(i,:) = desired_theta;
    i = mod(i+1,buffersize)+1
    
    % Plot
    plot(x)
    drawnow;
end

fclose(serialPort); 
delete(serialPort);