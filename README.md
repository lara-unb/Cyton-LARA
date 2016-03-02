# Cyton-LARA
Cyton Alpha 7D1G is a 7 DOF Robot Arm developed by Robai. It presents 9 servo motors controlled by a SSC-32 board. However, the servo motors does not provide position feedback. For this reason, the Cyton Alpha only allows open-loop projects, restricting a lot the possibilities of development. This project has as main goal closing the loop of the Cyton arm and developing a Matlab-Arduino user interface with some functions.

## Hardware Required
1. Cyton Alpha 7D1G
2. Arduino Mega

## Software Required
1. Arduino IDE
2. Matlab

## Closing the loop
In order to close the loop of the arm, it is necessary to get the position of each servo motor (only 7 of them). Each servo motor has a potentiometer inside. Somehow, most of them doesn't have a wire solded on it. The ideia is to open the servos and sold a wire on the potentiometer. Doing so, for each servo position, the potentiometer will show a different voltage. Also, it is possible to see that the voltage grows linearly according to the servo position. Obs.: do this at your own risk.

## Arduino Mega
The Cyton C++ API provided by Robai is good way to program the arm, however, as we are working with the feedback, the Arduino Mega is included in the hardware for reading the voltage values of the servos and also to execute the servo movement. How? Plugging a wire on the 

