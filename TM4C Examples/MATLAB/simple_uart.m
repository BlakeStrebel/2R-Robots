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
% Written By: Benjamen Lim & Huan
% Date: 1 Feb 2018
% 

clear;clc;
s = serial('COM5', 'BaudRate', 128000, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none');
%s.Terminator = 'CR'; 
fopen(s);

fwrite(s, 'b\n'); % Test by sending a char to the connected microcontroller
%reply = fscanf(s)
%reply = fread(s,4,'uchar')
reply = fread(s, 4, 'float32')
fclose(s);
%Afloat = typecast(reply , 'single') % I added it
command = 1;


% while true 
%     clc
%     disp('R2R Arm Interface')
%     disp('-----------------')
%     disp('System online.')
%     disp('You last entered: ')
%     disp(x);
%     x = input('Do not enter k: ','s');
%     if x == 'k'
%         break;
%     end
%     
% end