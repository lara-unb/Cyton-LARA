function [] = closeSerial()

fclose(instrfindall);
delete(instrfindall);