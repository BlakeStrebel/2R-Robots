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

pause(1);

for i = 1:100
    % update desired angle
    fprintf(Tiva_Serial,'%c\n','1');
    Currents(1:4,i) = fscanf(Tiva_Serial,'%d %d %d %d');
    pause(0.01);
end

M1_A = Currents(1,:);
M1_B = Currents(2,:);
M2_A = Currents(3,:);
M2_B = Currents(4,:);


figure;
subplot(2, 2, 1);
plot(M1_A);
title('Motor 1 Phase A');
subplot(2,2,2);
plot(M1_B);
title('Motor 1 Phase B');
subplot(2,2,3);
plot(M2_A);
title('Motor 2 Phase A');
subplot(2,2,4);
plot(M2_B);
title('Motor 2 Phase B');


fprintf('M1_A: %2.0f\n', mean(M1_A));
fprintf('M1_B: %2.0f\n', mean(M1_B));
fprintf('M2_A: %2.0f\n', mean(M2_A));
fprintf('M2_B: %2.0f\n', mean(M2_B));

fclose(Tiva_Serial);