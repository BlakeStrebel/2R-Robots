function robotReset(serial)
    fwrite(serial, 8);
    fwrite(serial, 26);
end