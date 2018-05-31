function success = initialPositionSend(p, serial)
fwrite(serial, 13);
fwrite(serial, p', 'float32');
success = [];
end