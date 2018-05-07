Tiva_port = 'COM5'; % Tiva board serial port

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% configure ports
Tiva_Serial = serial(Tiva_port, 'BaudRate', 115200, 'FlowControl', 'hardware','Timeout',15); 

fprintf('Opening ports %s\n',Tiva_port);

% opens serial connection
fopen(Tiva_Serial);

% closes serial port when function exits
%clean1 = onCleanup(@()fclose(Tiva_Serial));

% Crank up the position gain
fprintf(Tiva_Serial,'%c\n','h');
fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[.1,0,0]); % might need to play with this value

% Get motors close to zero position
fprintf(Tiva_Serial,'%c\n','f');
fprintf(Tiva_Serial, '%d %d\n',[0,0]);

fprintf(Tiva_Serial,'%c\n','j');

pause(5);

% Also might need to play with angle spacing
i = 1;
for angle = 0:1:16383 % Forward direction
    % send to the new position
    fprintf(Tiva_Serial,'%c\n','f');
    fprintf(Tiva_Serial, '%d %d\n',[angle,0]);
    pause(.1); % wait for motor to settle
   
    fprintf(Tiva_Serial,'%c\n','e'); % read desired angle
    desPos1(i) = fscanf(Tiva_Serial,'%d');
    desPos2(i) = fscanf(Tiva_Serial,'%d');
   
    fprintf(Tiva_Serial,'%c\n','a'); % read actual angle
    absPos1(i) = fscanf(Tiva_Serial,'%d');
    absPos2(i) = fscanf(Tiva_Serial,'%d');
   
    fprintf(Tiva_Serial,'%c\n','d'); % read pwm duty cycle
    pwm1(i) = fscanf(Tiva_Serial,'%d');
    pwm2(i) = fscanf(Tiva_Serial,'%d');

    clc;
    fprintf('Current Angle: %d',angle);
    i = i + 1; 
end
    
fclose(Tiva_Serial);