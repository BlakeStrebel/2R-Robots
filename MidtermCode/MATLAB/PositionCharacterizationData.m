Tiva_port = 'COM5'; % Tiva board serial port

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% configure ports
Tiva_Serial = serial(Tiva_port, 'BaudRate', 115200, 'FlowControl', 'hardware','Timeout',15); 

fprintf('Opening ports %s and %s....\n',Tiva_port);

% opens serial connection
fopen(Tiva_Serial);

% closes serial port when function exits
clean1 = onCleanup(@()fclose(Tiva_Serial));

% Get motors close to zero position
fprintf(Tiva_Serial,'%c\n','f');
fprintf(Tiva_Serial, '%d %d\n',0,0);
fprintf(Tiva_Serial,'%c\n','j');

% Crank up the position gain
fprintf(Tiva_Serial,'%c\n','h');
fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[10,0,0]); % might need to play with this value

% Also might need to play with angle spacing
i = 1;
for angle = 0:5:360 % Forward direction
    % send to the new position
    fprintf(Tiva_Serial,'%c\n','f');
    fprintf(Tiva_Serial, '%d %d\n',round(angle/360*163830),round(angle/360*163830));
    pause(2); % wait for motor to settle
   
    fprintf(Tiva_Serial,'%c\n','e'); % read desired angle
    desPos1(i) = fscanf(Tiva_Serial,'%d');
    desPos2(i) = fscanf(Tiva_Serial,'%d');
   
    fprintf(Tiva_Serial,'%c\n','a'); % read actual angle
    absPos1(i) = fscanf(Tiva_Serial,'%d');
    absPos2(i) = fscanf(Tiva_Serial,'%d');
   
    fprintf(Tiva_Serial,'%c\n','d'); % read pwm duty cycle
    pwm1 = fscanf(Tiva_Serial,'%d');
    pwm2 = fscanf(Tiva_Serial,'%d');

    i = i + 1; 
end
    
fclose(Tiva_Serial);