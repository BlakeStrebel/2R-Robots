% This is a basic interface to the TM4C controller through UART
% 
%
% Configured for 115200 Baud Rate
% Replace 'COM3' with the port that you see in Device Manager
% 
% The TM4C controller should reply with a carriage return character, which
% terminates the line and returns the fscan function. 
% 
% Once done, fclose closes the port. If you see the port and cannot
% connect to it, close and reopen MATLAB
%
% R2R
% Written By: Benjamen Lim
% Date: 1 Feb 2018
% 

clear;clc;
s = serial('COM10','BAUD',115200);
s.Terminator = 'CR'; 
fopen(s);
fwrite(s, 'b\n'); % Test by sending a char to the connected microcontroller
reply = fscanf(s)
fclose(s);
command = 1;

% while true 
%     clear
%     x = input('Do not enter k: ');
%     if x == 'k'
%         break;
%     end
%     print(x);
% end