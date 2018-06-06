function p = positionRead(motor, serial)
if motor == 1
    fwrite(serial, 2);
    p(1) = readFloat(serial);
    p(2) = p(1) / pi * 180;
elseif motor == 2
    fwrite(serial, 3);
    p(1) = readFloat(serial);
	p(2) = p(1) / pi * 180;
else 
    fwrite(serial, 4);
    p(1) = readFloat(serial);
    p(2) = p(1) / pi * 180;    
    p(3) = readFloat(serial);
    p(4) = p(3) / pi * 180;    
end
end