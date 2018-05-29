function array = readArray(row, column, serial)
array = [];
i = 0;
while i < row * column
    while serial.BytesAvailable >= 4
        array = [array; fread(serial, floor(serial.BytesAvailable / 4), 'float32')];
        i = i + 1;
    end
end
array = reshape(array, [row, column]);
end
