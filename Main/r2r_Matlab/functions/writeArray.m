function writeArray(s, array)
% This is a function writting a float array into the serial.
% Input:
% s: a serial port.
% array: a matrix,

[row, column] = size(array); 
total = row * column;
data = reshape(array, row * column, 1);
npertime = s.OutputBufferSize / 4;
times = floor(total / npertime);
for i = 1: times
    while s.BytesToOutput ~= 0
    end
    fwrite(s, data((i - 1) * npertime + 1: i * npertime), 'float32');
end
while s.BytesToOutput ~= 0
end
if times == 0
    fwrite(s, data(1: end), 'float32');
else
    fwrite(s, data((i + 1) * npertime + 1: end), 'float32');
end