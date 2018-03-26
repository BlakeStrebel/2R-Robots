% This is a basic interface to the TM4C controller through UART
% 
% Configured for 115200 Baud Rate
% Replace 'COM3' with the port that you see in Device Manager
% 
% The TM4C controller should reply with a carriage return character, which
% terminates the line and returns the fscan function. 
% 
% Once done, fclose closes the port. If you see the port and cannot
% connect to it, close and reopen MATLAB.
%
% The array should be loaded before starting the script.
%
% R2R
% Written By: Benjamen Lim & Huan
% Date: 1 Feb 2018
% 

% 
clear s j;
clc;
s = serial('COM19', 'BaudRate', 128000, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none');
s.Terminator = 'CR'; 
set(s,'timeout',1);
fopen(s);
j = onCleanup(@()fclose(s)); %closes serial on termination!

%Afloat = typecast(reply , 'single') % I added it
command = 1;
lengthOfArray = 4000;
Y = 0;
testarray(4000) = 0;
x = NaN;
level = 1;
lastx = NaN
currMode = 'Idle';


while true 
    clc
    disp('R2R Arm Interface')
    disp('-----------------')
    if isnan(x)
        dispLine = ['Current Mode: ', currMode];
        disp(dispLine)
        switch(lastx)
            case 'a'
                disp('Array loaded.')
            case 'b'
                disp('Gains loaded.')
                disp('Current Algorithm to run: PID')
            case 'c'
                disp('Custom controller setup complete.')
                disp('Current Algorithm to run: Custom')
            case 'i'
                disp('Current gains loaded.')
            case 'r'
                disp('Run Complete.')
            otherwise
                disp('System online.')
        end
        disp('a: Load Array           b: PID Control gains')
        disp('c: Custom Control       d: Other Control 2')
        disp('e: Other Control 3      g: Show gains')
        disp('i: Current gains        p: Go to position (angle)')
        disp('m: Get mode             r: Run')
        disp('k: End the program')
        disp('You last entered: ')
        disp(lastx);
        x = input('Enter option: ','s');
        continue;
    end
    % write the option to the MCU
    fwrite(s,x);
    if x == 'a'
        disp('Array input selected');
        disp('Protected inputs: s, x, j');
        disp('Array must already be specified in the MATLAB workspace')
        Y = input('Please specify array variable: ');
        lastx = 'a';
        x = NaN;
        continue;
    end
    if x =='c'
        disp('Custom control selected');
        fwrite(s,'c'); 
        Y = 'a';
        % Code to read reply 
        % FIX: 
        reply = fscanf(s);
        fscanf(s); % discard
        reply2 = fscanf(s);
        fscanf(s); % discard
        while reply ~= 'z'
            disp(reply);
            %disp(reply2);
            %disp('Here');
            %disp('here');
            switch strtrim(reply2)
                case 'num'
                    Y = input('Num: ','s');
                    Y = str2num(Y);
                    fwrite(s,Y,'float32');
                    
                case 's'
                    Y = input('Str:','s');
                    fwrite(s,Y);
                    
                case 'z'
                    break;
                otherwise
                    disp('No match found');
            end
            %  disp('debug2');
            %disp(reply);
            reply = fscanf(s);
            fscanf(s);
            reply2 = fscanf(s);
            fscanf(s);
%             disp(reply);
%             strtrim(reply)
%             if strtrim(reply)=='z'
%                 pause(10);
%                 break;
%             else
%                 fscanf(s)
%                 reply2 = fscanf(s)
%                 fscanf(s)
%             end
%             
            %reply = fscanf(s)
            
        end
        lastx = 'c';
        x = NaN;
        continue;

    end
    % Position gains
    if x == 'b'
        disp('PID control')
        Y = input('Enter gains Kp Ki Kd: ','s');
        Y = str2num(Y) % convert to array
        %fwrite(s,'b'); % tell the micro we are using option b (PID)
        fwrite(s,Y,'float32') %write to micro our PID array
        disp('Written to microcontroller: ')
        fread(s,3,'float32')
        pause(3);
      
        
        % Do something to the gains
        lastx = 'b';
        x = NaN;
        continue;
    end
    % Show PID gains
    if x == 'g'
        disp('Current set gains are: ');
        fread(s,3,'float32')
        lastx = 'g';
        x = NaN;
        pause(3);
        continue;
    end
    % Current gains
    if x == 'i'
        disp('PID control')
        Y = input('Enter gains Kp Ki Kd: ','s');
        Y = str2num(Y) % convert to array
        %fwrite(s,'b'); % tell the micro we are using option b (PID)
        fwrite(s,Y,'float32') %write to micro our PID array
        disp('Written to microcontroller: ')
        fread(s,3,'float32') 
        pause(3);
        % Do something to the gains
        lastx = 'i';
        x = NaN;
        continue;
    end
    
    % Go to position
    if  x == 'p'
        disp('Setting Hold position')
        Y = input('Position (angle): ');
        fwrite(s,Y,'float32') %write to micro our PID array
        currMode = ['Hold ','(',num2str(Y),')'];
        lastx = 'p';
        x = NaN;
        continue;
    end
    
    % Get mode (are we in tracking (PID)/idle/custom? - when you hit 'r', it
    % will run this mode)
    if x == 'm'
        disp('Current mode is:')
        tryRead=fread(s)
        if (isstring(tryRead))
            currMode = tryRead;
        else
            currMode = 'Error!';
        end
        pause(2);
        lastx = 'm';
        x = NaN;
        continue;
    end
        

    if x == 'r'
        % Just to receive data
        % h = waitbar(0,'Running model...');
        % For live plotting
        x = 0 ;
        startSpot = 0;
        step = 1 ; % lowering step has a number of cycles and then acquire more data
        storedArray = zeros(lengthOfArray);
        % Send your variables here. 
        fwrite(s, 'b\n'); % Test by sending a char to the connected microcontroller
        
        for t = 1:lengthOfArray
            % For data receiving.
            %waitbar(t/lengthOfArray);
            %reply = fscanf(s)
            %reply = fread(s,4,'uchar')
            
            reply = fread(s, 1, 'float32'); %read 4 bytes and convert to float
            % For live plotting
            storedArray(t) = reply;
            x = [ x, reply ];
            plot(x) ;
            if ((t/step)-500 < 0)
                startSpot = 0;
            else
                startSpot = (t/step)-500;
            end
            axis([startSpot, (t/step+50), 0 , 300 ]);
            grid
            drawnow;
            
            %pause(0.1); %wait for 0.1 sec but this is wher we do the computations
        end
        close(h)
        lastx = 'r';
        x = NaN;
        continue
    end
    % Set current gains

    if x == 'k'
        fclose(s);
        disp('Goodbye.')
        break; 
    end
    
end