function matrix = readArray(s, row, column)
% This is a function reading a float array from the serial.
% Input:
% s: a serial port,
% rwo, column: the dimension of an array. 

matrix = [];
total = row * column * 4;
while s.BytesAvailable ~= 0
    matrix = [matrix; fread(s, s.BytesAvailable / 4, 'float32')];
    pause(0.5)
end
matrix = reshape(matrix, row, column); %read 4 bytes and convert to float. % read it colomnwise.
end