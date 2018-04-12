function writeArray(s, array)
% This is a function writting a float array into the serial.
% Input:
% s: a serial port.
% array: a matrix,

[row, column] = size(array);   
fwrite(s, reshape(array, row * column, 1), 'float32'); % write vector only.
end