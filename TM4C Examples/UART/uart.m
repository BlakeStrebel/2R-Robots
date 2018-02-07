% This is a basic command line list to communicate with TIVA.
% 
%
% Configured for 115200 Baud Rate
% Replace 'COM5' with the port that you see in Device Manager
% 
% This file works along with the uart.c file. 
% 
% PLEASE RUN THIS FILE LINE BY LINE !!
%
% R2R
% Written By: Huan Weng
% Date: 7 Feb 2018
% 

clear;clc;
s = serial('COM5', 'BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none');
fopen(s);
s
reply = fscanf(s,'%c',16)
fprintf(s,'%c','b');
s
reply = fscanf(s,'%c',1)
%Repeat if you like.
fclose(s);