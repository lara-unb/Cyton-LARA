# Cyton-LARA
Cyton Alpha 7D1G is a 7 DOF Robot Arm developed by Robai. It presents 9 servo motors controlled by a SSC-32 board. However, the servo motors does not provide position feedback. For this reason, the Cyton Alpha only allows open-loop projects, restricting a lot the possibilities of development. This project has as main goal closing the loop of the Cyton arm and developing a Matlab-Arduino user interface with some functions.

## Hardware Required
1. Cyton Alpha 7D1G
2. Arduino Mega

## Software Required
1. Arduino IDE
2. Matlab

## Hardware Modification - Closing the Loop
In order to close the loop of the arm, it is necessary to get the position of each servo motor (only 7 of them). Each servo motor has a potentiometer inside. Somehow, most of them doesn't have a wire solded on it. The ideia is to open the servos and sold a wire on the potentiometer. Doing so, for each servo position, the potentiometer will show a different voltage. Also, it is possible to see that the voltage grows linearly according to the servo position. Obs.: Do this at your own risk.

## Arduino Mega
The Cyton C++ API provided by Robai is good way to program the arm, however, as we are working with the feedback, the Arduino Mega is included in the hardware for reading the servo voltage values and also to execute the servo movement. How? Plugging a wire on the Arduino TX port on the SSC-32 RX port will allow to communicate both boards through Serial. Also, it is needed to plug a wire from Arduino Ground to the SSC-32 ground. It is recommended to set the baud rate of SSC-32 to 9600 (just disconnect to first jumper located in the BAUD section of the board). And one final detail: connect the Arduino AREF port tot he 3.3 V port in order to augment the servo feedback resolution.

## Matlab
The Matlab is mainly used to calculate the Forward Kinematics of the arm, as the Arduino is not powerful enough to do so. Also, it is used to present the User Interface with a Menu with some options.

## Arduino Libraries Required
1. SSC32: http://blog.martinperis.com/2011/05/libssc32-arduino-ssc32.html
2. Filters: https://github.com/JonHub/Filters
3. StandardCplusplus: https://github.com/maniacbug/StandardCplusplus

## Maltab Toolbox Required
1. DQ Robotics: http://dqrobotics.sourceforge.net/

## DH Parameters
theta =  [0, 0, 0, 0, pi, pi/2, 0]  
d =     [0.0379, -0.0046, 0.145, -0.011, 0.175, 0.0074, 0.0]  
a =     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0677]  
alpha = [0, pi/2, -pi/2, pi/2, -pi/2, pi/2, pi/2]  

## Matlab Menu
The file CytonMatlab.m is the main program that initialize the User Interface and also calls the other functions created. The Menu looks as shown below:
				 
		0. Connect to Arduino
		1. Initial Position
		2. Forward Kinematics
		3. Record
		4. Replay
		5. Change Desired Position (Manual)
		6. Simulation (Forward Kinematics)
		7. Position Feedback
		8. Help
		9. Close
0. Connect to Arduino: Initialize the Serial Port.
1. Initial Position: Set all the Servos to 0 degrees (Vertical Position).
2. Forward Kinematics: According to the desired position, a vector of angles is calculated by Forward Kinematics and it is sent to the Arduino via Serial several times until the Arm reaches the final position. The Arduino receives each vector of angles. and send a movement command to the arm also via Serial.
3. Record: Record the movement of the arm through time.
4. Replay: Repeat the movement recorded. It is recommended to set the arm to the initial position before executing the Replay option.
5. Change Desired Position: Changes the desired position used in Option 2 (Forward Kinematics) and 6 (Simulation).
6. Simulation: Simulates the Forward Kinematics according to the desired position. It is recommended to simulates the Forward Kinematics first before executing it on the robot.
7. Position Feedback: Returns the angle position of each servo of the arm.
8. Help: Shows the functionalities of each option.
9. Close: Closes the Serial Port and also the program.
