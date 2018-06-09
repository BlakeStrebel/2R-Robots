Tiva_port = 'COM5'; % Tiva board serial port
DECIMATION = 10;
PWMPERIOD = 4000;

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% configure ports
Tiva_Serial = serial(Tiva_port, 'BaudRate', 115200,'FlowControl', 'hardware', 'Timeout',5); %,

fprintf('Opening port %s....\n',Tiva_port);

% opens serial connection
fopen(Tiva_Serial);

pause(1);

% Update gains
fprintf(Tiva_Serial, '%c\n','h');
fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[3.5,0,0.5]);
pause(0.5);

%% Load First Trajectory
trajectory1 = [0,0;1,360;2,0;3,0;4,0];
trajectory2 = [0,0;1,360;3,-360;4,0];

[ref1, ref2] = load_trajectory(Tiva_Serial, trajectory1, trajectory2);

has_quit = false;

while (~has_quit)
   test = input('Press <Enter> when ready or q to quit: ','s');
   
   if (strcmp(test, 'q'))
       has_quit = 1;
   else
       fprintf(Tiva_Serial,'%c\n','l');  
       read_plot_matrix(Tiva_Serial, ref1(1:DECIMATION:end)', ref2(1:DECIMATION:end)');
   end
end

%% Load Second Trajectory
fprintf('Loading a new trajectory...\n');

trajectory1 = [0,0;1,180;3,-180;4,0];
trajectory2 = [0,0;1,-180;3,180;4,0];

[ref1, ref2] = load_trajectory(Tiva_Serial, trajectory1, trajectory2);

has_quit = false;

while (~has_quit)
   test = input('Press <Enter> when ready or q to quit: ','s');
   
   if (strcmp(test, 'q'))
       has_quit = 1;
   else
       fprintf(Tiva_Serial,'%c\n','l');  
       read_plot_matrix(Tiva_Serial, ref1(1:DECIMATION:end)', ref2(1:DECIMATION:end)');
   end
end

%% Load Third Trajectory
fprintf('Switch robot configuration now...\n');


% Update gains
fprintf(Tiva_Serial, '%c\n','h');
fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[1,0,0]);
pause(0.5);

trajectory1 = [0,0;1,180;2,-180;3,180;4,0];
trajectory2 = [0,0;1,-180;2,180;3,-180;4,0];

[ref1, ref2] = load_trajectory(Tiva_Serial, trajectory1, trajectory2);

has_quit = false;

while (~has_quit)
   test = input('Press <Enter> when ready or q to quit: ','s');
   
   if (strcmp(test, 'q'))
       has_quit = 1;
   else
       fprintf(Tiva_Serial,'%c\n','l');  
       read_plot_matrix(Tiva_Serial, ref1(1:DECIMATION:end)', ref2(1:DECIMATION:end)');
   end
end

fclose(Tiva_Serial);



%% Helper

function [ref1, ref2] = load_trajectory(Tiva_Serial, trajectory1, trajectory2)

fprintf('Loading Trajectory....\n');
fprintf(Tiva_Serial,'%c\n','k');  

ref1 = genRef(trajectory1,'cubic');
ref1 = round(ref1/360*16383);

ref2 = genRef(trajectory2,'cubic');
ref2 = round(ref2/360*16383);

% Motor 1
fprintf(Tiva_Serial,'%d\n',size(ref1,2));   % Send number of samples to Tiva
for i = 1:size(ref1,2)                   % Send trajectory to Tiva
   fprintf(Tiva_Serial,'%d\n',ref1(i)); 
end

% Motor 2
fprintf(Tiva_Serial,'%d\n',size(ref2,2));   % Send number of samples to Tiva
for i = 1:size(ref2,2)                   % Send trajectory to Tiva
   fprintf(Tiva_Serial,'%d\n',ref2(i)); 
end


end
