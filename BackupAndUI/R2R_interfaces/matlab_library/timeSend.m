function success = timeSend(t, serial)
fwrite(serial, 9);
fwrite(serial, t, 'int');
success = [];
end