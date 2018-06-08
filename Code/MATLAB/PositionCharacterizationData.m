% PositionCharacterizationData.m is used to obtain data used to
% characterize the position based torque variation in a BLDC motor as a
% result of magnetic cogging forces.

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

% set the position gain
Kp = 0.5;
fprintf(Tiva_Serial,'%c\n','h');
fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[Kp,0,0]); % might need to play with this value

% send motors to starting position
fprintf(Tiva_Serial,'%c\n','f');
fprintf(Tiva_Serial, '%d %d\n',[0,0]);

% set mode to hold desired position
fprintf(Tiva_Serial,'%c\n','j');
fprintf('Moving to starting position\n');

pause(5);

fprintf('Here we go\n');

i = 1;
for pos = 0:4:16383 % Forward direction
    % update desired angle
    fprintf(Tiva_Serial,'%c\n','f');
    fprintf(Tiva_Serial, '%d %d\n',[pos,pos]);
   
    fprintf(Tiva_Serial,'%c\n','e'); % read desired angle
    desPos1(i) = fscanf(Tiva_Serial,'%d');
    desPos2(i) = fscanf(Tiva_Serial,'%d');
   
    fprintf(Tiva_Serial,'%c\n','a'); % read actual angle
    absPos1(i) = fscanf(Tiva_Serial,'%d');
    absPos2(i) = fscanf(Tiva_Serial,'%d');
   
    fprintf(Tiva_Serial,'%c\n','1'); % read current
    current1(i) = fscanf(Tiva_Serial,'%d');
    current2(i) = fscanf(Tiva_Serial,'%d');

    clc;
    fprintf('Current Angle: %d',pos);
    i = i + 1; 
end
    
fclose(Tiva_Serial);

figure;
plot(desPos1, current1);
title('Motor 1: Current vs. Desired Position');
xlabel('Position (counts)');
ylabel('Current (counts)');

figure;
plot(desPos2, current2);
title('Motor 2: Current vs. Desired Position');
xlabel('Position (counts)');
ylabel('Current (counts)');



