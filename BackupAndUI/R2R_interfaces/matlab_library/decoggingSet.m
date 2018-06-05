function success = decoggingSet(value, serial)
if value == 1
    fwrite(serial, 28);
else
    fwrite(serial, 29);
end