function Client()
%   provides a menu for interfacing with hopper robot system
Tiva_port = 'COM5'; % Tiva board serial port
DECIMATION = 1;
PWMPERIOD = 4000;

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% configure ports
Tiva_Serial = serial(Tiva_port, 'BaudRate', 115200, 'FlowControl', 'hardware','Timeout',15); 

fprintf('Opening port %s....\n',Tiva_port);

% opens serial connection
fopen(Tiva_Serial);

% closes serial port when function exits
% clean1 = onCleanup(@()fclose(Tiva_Serial));

% globals
has_quit = false;

% menu loop
while ~has_quit
    % read the user's choice
    disp('a: Read Absolute Angle  b: Read Relative Angle')
    disp('c: Set PWM              d: Read PWM')
    disp('e: Read Desired Angle   f: Set Desired Angle')
    disp('g: Set mode             h: Set PID gains')
    disp('i: Get PID gains        j: Hold Position')
    disp('k: Set trajectory       l: Execute Trajectory ')
    disp('m: Move Arm             n: Load and Run Test')
    disp('o: Set Decogging        r: Zero Encoders')
    disp('q: Quit')
    selection = input('\nENTER COMMAND: ', 's');
   
    fprintf(Tiva_Serial,'%c\n',selection);  % send the command to the Tiva
    
    % take the appropriate action
    switch selection
        case 'a'
            absPos1 = fscanf(Tiva_Serial,'%d');
            absPos2 = fscanf(Tiva_Serial,'%d');
            fprintf('The absolute motor angles are:\nMotor 1: %3.2f degrees Motor 2: %3.2f degrees\n',absPos1/16383*360,absPos2/16383*360);
        case 'b'
            relPos1 = fscanf(Tiva_Serial,'%d');
            relPos2 = fscanf(Tiva_Serial,'%d');
            fprintf('The relative motor angles are:\nMotor 1: %3.2f degrees Motor 2: %3.2f degrees\n',relPos1/16383*360,relPos2/16383*360);
        case 'c'
            PWM1 = input('Enter your desired motor1 PWM [-100,100]:  ');
            PWM2 = input('Enter your desired motor2 PWM [-100,100]:  ');
            fprintf(Tiva_Serial, '%d %d\n',[PWM1/100*(PWMPERIOD), PWM2/100*(PWMPERIOD)]);
            fprintf('PWM1 set to %3.2f\nPWM2 set to %3.2f\n',PWM1,PWM2);
        case 'd'
            pwm1 = fscanf(Tiva_Serial,'%d');
            pwm2 = fscanf(Tiva_Serial,'%d');
            fprintf('The  motor duty cycles are:\nMotor 1: %3.2f\nMotor 2: %3.2f\n',pwm1/(PWMPERIOD)*100, pwm2/(PWMPERIOD)*100);
        case 'e'
            desPos1 = fscanf(Tiva_Serial,'%d');
            desPos2 = fscanf(Tiva_Serial,'%d');
            fprintf('The desired motor angles are:\nMotor 1: %3.2f degrees\nMotor 2: %3.2f degrees\n',desPos1/16383*360,desPos2/16383*360);
        case 'f'
            fprintf('Enter desired motor angles in degrees: \n');
            desPos1 = input('Motor1: ');
            desPos2 = input('Motor2: ');
            fprintf(Tiva_Serial, '%d %d\n',[round(desPos1/360*16383), round(desPos2/360*16383)]);
        case 'g'
            mode = fscanf(Tiva_Serial,'%d');
            if mode == 0
                fprintf('The controller is in IDLE.\n');
            elseif mode == 1
                fprintf('The controller is in HOLD.\n');
            elseif mode == 2
                fprintf('The controller is in TRACK.\n');
            end  
        case 'h'
            Kp = input('Enter your desired Kp position gain: ');
            Ki = input('Enter your desired Ki position gain: ');
            Kd = input('Enter your desired Kd position gain: ');
            fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[Kp,Ki,Kd]);
        case 'i'
            Kp = fscanf(Tiva_Serial, '%f');    
            Ki = fscanf(Tiva_Serial, '%f');   
            Kd = fscanf(Tiva_Serial, '%f');    
            fprintf('The controller is using Kp = %3.2f, Ki = %3.2f, and Kd = %3.2f.\n',[Kp,Ki,Kd]);
        case 'j'
            fprintf('The motors are now holding their positions');
        case 'k'
            error_check = 1;
            while (error_check)
                fprintf('Motor 1:\n');
                mode = input('    Enter mode (''linear'', ''cubic'', ''step'', ''zero''): ');
                if (strcmp(mode, 'zero'))
                    time = input('Enter time in seconds: ');
                    ref1 = zeros(time*1000,1)';
                else
                    trajectory = input('Enter position trajectory, in sec and degrees [time1, pos1; time2, pos2; ...]:');
                    ref1 = genRef(trajectory,mode);
                    ref1 = round(ref1/360*16383);
                end
               
                fprintf('Motor 2:\n');
                mode = input('    Enter mode (''linear'', ''cubic'', ''step'', ''zero''): ');
                if (strcmp(mode,'zero'))
                    time = input('Enter time in seconds: ');
                    ref2 = zeros(time*1000,1)';
                else
                    trajectory = input('Enter position trajectory, in sec and degrees [time1, pos1; time2, pos2; ...]:');
                    ref2 = genRef(trajectory,mode);
                    ref2 = round(ref2/360*16383);
                end
                
                if (size(ref1,2) == size(ref2,2))
                    error_check = 0;
                else
                    fprintf('Your reference trajectories are different lengths, try again');
                end
                
            end
           
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
        case 'l'
            read_plot_matrix(Tiva_Serial, ref1(1:DECIMATION:end)', ref2(1:DECIMATION:end)');
        case 'm'
            lengthofTime = input('Time to run for (s): ');
            fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[0,0,0]); % sets gains to zero
            ref1 = genRef([0,0;lengthofTime,0],'linear');
            ref1 = round(ref1/360*16383);
            ref2 = ref1;
            fprintf(Tiva_Serial,'%d\n',size(ref1,2));   % Send number of samples to Tiva
            for i = 1:size(ref1,2)                   % Send trajectory to Tiva
               fprintf(Tiva_Serial,'%d\n',ref1(i)); 
            end
            
            % Motor 2
            fprintf(Tiva_Serial,'%d\n',size(ref2,2));   % Send number of samples to Tiva
            for i = 1:size(ref2,2)                   % Send trajectory to Tiva
               fprintf(Tiva_Serial,'%d\n',ref2(i)); 
            end
        case 'n'
            fprintf(Tiva_Serial, '%3.2f %3.2f %3.2f\n',[2,0,0]);
            fprintf('Gains Set to [2,0,0].');
            pause(1);
            ref1 = genRef([0,0;1,180;2,360;3.25,90;4,0;4.5,90;5,0],'cubic');
            ref1 = round(ref1/360*16383);
            ref2 = ref1;
            
            %Motor 1
            fprintf(Tiva_Serial,'%d\n',size(ref1,2));   % Send number of samples to Tiva
            for i = 1:size(ref1,2)                   % Send trajectory to Tiva
               fprintf(Tiva_Serial,'%d\n',ref1(i)); 
            end
            
            % Motor 2
            fprintf(Tiva_Serial,'%d\n',size(ref2,2));   % Send number of samples to Tiva
            for i = 1:size(ref2,2)                   % Send trajectory to Tiva
               fprintf(Tiva_Serial,'%d\n',ref2(i)); 
            end
            fprintf('Trajectory Sent');
            
            read_plot_matrix(Tiva_Serial, ref1(1:DECIMATION:end)', ref2(1:DECIMATION:end)');
        case 'r'
            fprintf('Encoders zeroed\n');
        case 'q'
            has_quit = 1;
        case '1'
            counts(1:4) = fscanf(Tiva_Serial, '%d %d %d %d');
            fprintf('Motor 1 A: %d, B: %d\nMotor2 A: %d, B: %d\n',counts(1), counts(2), counts(3), counts(4)); 
        case '2'
            mA(1:4) = fscanf(Tiva_Serial, '%d %d %d %d');
            fprintf('Motor 1 A: %d, B: %d\nMotor2 A: %d, B: %d\n',mA(1), mA(2), mA(3), mA(4)); 
        case '3'
           read_plot_matrix_current(Tiva_Serial);
        case '4'
            Kp = input('Enter your desired Kp position gain: ');
            Ki = input('Enter your desired Ki position gain: ');
            fprintf(Tiva_Serial, '%3.2f %3.2f\n',[Kp,Ki]);
        case '5'
            Kp = fscanf(Tiva_Serial, '%f');    
            Ki = fscanf(Tiva_Serial, '%f');     
            fprintf('The controller is using Kp = %3.2f and Ki = %3.2f\n',[Kp,Ki]);
        otherwise
            fprintf('Invalid Command, try again...\n');
    end
end

fclose(Tiva_Serial);


end