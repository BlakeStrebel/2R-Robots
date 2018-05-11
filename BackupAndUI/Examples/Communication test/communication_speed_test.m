% This is a control speed test program. 
%
% Configured for 128000 Baud Rate
% Replace 'COM5' with the port that you see in Device Manager
% 
% This file works along with the uart.c file. 
%
% R2R
% Written By: Huan Weng
% Date: 2 Mar 2018
% 
clear;clc;
reply = [0.0,0.1,0.2,0.3,0.4;0.5,0.6,0.7,0.8,0.9;1.0,1.1,1.2,1.3,1.4;1.5,1.6,1.7,1.8,1.9];
s = serial('COM5', 'BaudRate', 128000, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none');
fopen(s);
tic;
%reply = fscanf(s,'%c',16)
while reply(1) < 109
    fwrite(s, reshape(reply, 20, 1), 'float32'); % write vector only.
    %s
    reply = reshape(fread(s, 20, 'float32'),4,5) + 1; %read 4 bytes and convert to float. % read it colomnwise.
end
t = toc;
reply
t = 100 / t
%Repeat if you like.
fclose(s);