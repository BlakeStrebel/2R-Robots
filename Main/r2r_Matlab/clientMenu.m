% This is a basic interface to the TM4C controller through UART
% 
% Configured for 115200 Baud Rate
% Replace 'COM3' with the port that you see in Device Manager
% 
% The TM4C controller should reply with a carriage return character, which
% terminates the line and returns the fscan function. 
% 
% Once done, fclose closes the port. If you see the port and cannot
% connect to it, close and reopen MATLAB.
%
% The array should be loaded before starting the script.
%
% R2R
% Written By: Benjamen Lim & Huan
% Date: 1 Feb 2018
% 

% 
clear all;
clc;
addpath('functions');

userserial = serialConfig('COM7');
fopen(userserial);
finishup = onCleanup(@()fclose(userserial)); %closes serial on termination!

dashnum = 20;
steplog = ['a'];
currentstep = 'a';
data = [];
csvfilen = 0;
defaulttraj = ones(500, 1);


while true
    clc
    disp('R2R Arm Interface')
    disp(char(ones(1,dashnum) * '-'))
    for i = 1: length(steplog) - 1
        successInfo(steplog, i, data);
        disp(char(ones(1, dashnum - i * 2) * '-'))
    end
    options = menuLib(steplog);
    if strcmp(steplog, 'q')
        break
    end
    response = input('Enter value(s): ', 's');
    [steplog, data] = inputProcess(steplog, options, response, data);    
    
    matrix = firmwareControl(userserial, steplog, data);
    
    
end
