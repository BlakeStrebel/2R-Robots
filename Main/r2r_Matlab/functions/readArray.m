function matrix = readArray(s, row, column)
% This is a function reading a float array from the serial.
% Input:
% s: a serial port,
% rwo, column: the dimension of an array. 

matrix = [];
total = row * column * 4;
data_rem = rem(total, s.InputBufferSize);
while s.BytesAvailable == 0
end
for i = 1: floor(total / s.InputBufferSize)
    matrix = [matrix; fread(s, s.InputBufferSize / 4, 'float32')];
end
if data_rem > 0
    matrix = [matrix; fread(s, data_rem / 4, 'float32')];
end
size(matrix)
matrix = reshape(matrix, row, column); %read 4 bytes and convert to float. % read it colomnwise.
end