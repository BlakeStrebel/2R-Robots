function Client()
%   provides a menu for interfacing with hopper robot system
Tiva_port = 'COM5'; % Tiva board serial port
DECIMATION = 10;

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

% globals
has_quit = false;

% menu loop
while ~has_quit
    % read the user's choice
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
            fprintf(Tiva_Serial, '%d %d\n',[PWM1/100*9600, PWM2/100*9600]);
            fprintf('PWM1 set to %3.2f\nPWM2 set to %3.2f\n',PWM1,PWM2);
        case 'd'
            pwm1 = fscanf(Tiva_Serial,'%d');
            pwm2 = fscanf(Tiva_Serial,'%d');
            fprintf('The  motor duty cycles are:\nMotor 1: %3.2f%\nMotor 2: %3.2f%\n',pwm1/9600*100, pwm2/9600*100);
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
                mode = input('    Enter mode (''linear'', ''cubic'', ''step'': ');
                trajectory = input('Enter position trajectory, in sec and degrees [time1, pos1; time2, pos2; ...]:');
                ref1 = genRef(trajectory,mode);
                ref1 = round(ref1/360*16383);
                ref2 = ref1;
                error_check = 0;
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
        case 'q'
            has_quit = 1;
        otherwise
            fprintf('Invalid Command, try again...\n');
    end
end

fclose(Tiva_Serial);


end