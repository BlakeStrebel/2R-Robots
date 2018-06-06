function float = readFloat(serial)
while serial.BytesAvailable < 4
end
float = fread(serial, 1, 'float32');
end