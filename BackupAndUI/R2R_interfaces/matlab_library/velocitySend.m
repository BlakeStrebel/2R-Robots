function success = velocitySend(v, motor, serial)
success = [];
if motor == 1
    fwrite(serial, 5);
    fwrite(serial, v(1), 'float32');
elseif motor == 2
    fwrite(serial, 6);
    fwrite(serial, v(2), 'float32');
elseif motor == 3
    fwrite(serial, 7);
    fwrite(serial, v', 'float32');
else
    fwrite(serial, 8);
end
end