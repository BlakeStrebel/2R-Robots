function data = positionDefault(motor)
if motor == 1
    data(1) = 0;
    data(2) = data(1) / pi * 180;
elseif motor == 2
    data(1) = 0;
	data(2) = data(1) / pi * 180;
else 
    data(1) = 0;
    data(2) = data(1) / pi * 180;
    data(3) = 0;
    data(4) = data(3) / pi * 180;    
end