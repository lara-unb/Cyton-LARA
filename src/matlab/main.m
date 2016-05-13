% Universidade de Brasília
% Laboratório de Automação e Robótica
% Autor: De Hong Jung
% Programa: Interface com usuário para execução de funções para o braço
%           robótico Cyton Alpha 7D1G
%           Comunicação com Arduino

clear all
clc

%============ Setup ================

desired_theta = [0,0,0,-45,0,45,0]';
feedback = ones(7,1);

%========= User Interface ==========

disp ('****** CYTON LARA *******');
disp ('======== Menu =========');
disp (' ');
disp ('0. Connect to Arduino');
disp ('1. Initial Position');
disp ('2. Forward Kinematics');
disp ('3. Record');
disp ('4. Replay');
disp ('5. Change Desired Position (Manual)');
disp ('6. Simulation (Forward Kinematics)');
disp ('7. Position Feedback');
disp ('8. Help');
disp ('9. Close');
disp (' ');
in = input('Option: ');

while (in < 9)
    %=========== Connection =============
    
    if in == 0
        % Windows
        arduino = serial('COM9','BaudRate',9600);

        % Linux        
        %arduino = serial('/dev/ttyUSB0','BaudRate',9600);     % sudo chmod 777 /dev/ttyUSB0

        fopen(arduino);
    end
    
    %======== Initial Position ==========

    if in == 1
        fprintf(arduino, '%d', in);
        pause(1);
        str = fgetl(arduino);

    %======= Forward Kinematics =========

    elseif in == 2
        fprintf(arduino, '%d', in);
        disp('Executing...');
        pause(1.0);
        CytonKinematics(arduino, deg2rad(desired_theta));
        
    
    %========== Shadow Clone ============
    
    elseif in == 3
        fprintf(arduino, '%d', in);
        clc;
        disp('Recording...');
        disp('Manually move the robot arm to any position');
        disp('Press any button to stop: ');
        in = input('');
        fprintf(arduino, '%d', in);
        
        clc;
        disp('==== Record complete ====');
        disp (' ');
        in = input('Press any button to continue: ');
    
    %============ Replay ===============
    
    elseif in == 4
        fprintf(arduino, '%d', in);
        disp ('Executing...');
        pause(4);
    
    %========== Desired Position ==========
    
    elseif in == 5
       fprintf(arduino, '%d', in);
       pause(3);
       aux = 1;

       str = fgetl(arduino);
       str = strtrim(str);
       str = strsplit(str, ' ');
       for n = 1:7
          if n == 2
            desired_theta(n) = -str2double(str{n});
          else
            desired_theta(n) = str2double(str{n});
          end
       end
        
    
    %=========== Simulation ==============
    
    elseif in == 6
        CytonSimulation(deg2rad(desired_theta));
        pause(1);
    
    
    %=========== Feedback ================
    
    elseif in == 7
       fprintf(arduino, '%d', in);
       pause(3);
       str = fgetl(arduino);
       str = strtrim(str);
       str = strsplit(str, ' ');
%        for n = 1:7
%           feedback(n) = str2double(str{n}); 
%           disp (feedback(n))
%        end
       clc;
       disp ('Servo Feedback: ');
       disp (str);
       in = input('Press any button to continue: ');
    
       
    %============== Help ================   
       
    elseif in == 8
        clc;
        disp ('================= INSTRUCTIONS =====================');
        disp ('0. Connect to Arduino: Initialize the Serial Port.');
        disp ('1. Initial Position: Set all the Servos to 0 degrees (Vertical Position).');
        disp ('2. Forward Kinematics: According to the desired position, a vector of angles is calculated by Forward Kinematics and it is sent to the Arduino via Serial several times until the Arm reaches the final position. The Arduino receives each vector of angles. and send a movement command to the arm also via Serial.');
        disp ('3. Record: Record the movement of the arm through time.');
        disp ('4. Replay: Repeat the movement recorded. It is recommended to set the arm to the initial position before executing the Replay option.');
        disp ('5. Change Desired Position: Changes the desired position used in Option 2 (Forward Kinematics) and 6 (Simulation).');
        disp ('6. Simulation: Simulates the Forward Kinematics according to the desired position. It is recommended to simulates the Forward Kinematics first before executing it on the robot.');
        disp ('7. Position Feedback: Returns the angle position of each servo of the arm.');
        disp ('8. Help: Shows the functionalities of each option.');
        disp ('9. Close: Closes the Serial Port and also the program.');
        in = input(' ');
    end
    
    %========= User Interface ==========
    
    clc;
    disp ('****** CYTON LARA *******');
    disp ('======== Menu =========');
    disp (' ');
    disp ('0. Connect to Arduino');
    disp ('1. Initial Position');
    disp ('2. Forward Kinematics');
    disp ('3. Record');
    disp ('4. Replay');
    disp ('5. Change Desired Position (Manual)');
    disp ('6. Simulation (Forward Kinematics)');
    disp ('7. Position Feedback');
    disp ('8. Help');
    disp ('9. Close');
    disp (' ');
    in = input('Option: ');
end

%============ Close ===============

fclose(arduino);
closeSerial();
