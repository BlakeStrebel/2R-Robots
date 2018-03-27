function matrix = readfloats(s, row, column)
% This is a function reading a float array from the serial.
% Input:
% s: a serial port,
% rwo, column: the dimension of an array. 

matrix = reshape(fread(s, row * column, 'float32'),row, column); %read 4 bytes and convert to float. % read it colomnwise.
end