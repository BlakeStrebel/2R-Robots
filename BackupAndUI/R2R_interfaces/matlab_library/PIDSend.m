function success = PIDSend(PID, motor, serial)
if motor == 1
    fwrite(serial, 17);
    fwrite(serial, PID(1), 'float32');
    fwrite(serial, PID(3), 'float32');
    fwrite(serial, PID(5), 'float32');
elseif motor == 2
    fwrite(serial, 18);
    fwrite(serial, PID(2), 'float32');
    fwrite(serial, PID(4), 'float32');
    fwrite(serial, PID(6), 'float32');
else
    fwrite(serial, 19);
    for i = 1: 6
        fwrite(serial, PID(i)', 'float32');
    end
end








success = [];
end