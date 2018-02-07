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
testarray(200) = 0;
x = NaN;
level = 1;
lastx = NaN
while true 
    clc
    disp('R2R Arm Interface')
    disp('-----------------')
    if isnan(x)
        switch(lastx)
            case 'a'
                disp('Array loaded.')
            case 'b'
                disp('Gains loaded.')
            case 'r'
                disp('Run Complete.')
            otherwise
                disp('System online.')
        end
        disp('a: Load Array           b: PID Control')
        disp('c: Other Control 1      d: Other Control 2')
        disp('e: Other Control 3      f: Run')
        disp('k: End the program')
        disp('You last entered: ')
        disp(lastx);
        x = input('Enter option: ','s');
        continue;
    end
    if x == 'a'
        disp('Array input selected');
        Y = input('Please specify array variable: ');
        lastx = 'a';
        x = NaN;
        continue;
        
    end
    if x == 'b'
        disp('PID control')
        Y = input('Enter gains [Kp Ki Kd]: ');
        lastx = 'b';
        x = NaN;
        continue;
    end
    if x == 'f'
        h = waitbar(0,'Running model...');
        for c = 1:100
            waitbar(c/100);
            pause(0.1); %wait for 0.1 sec but this is wher we do the computations
        end
        close(h)
        lastx = 'r';
        x = NaN;
        continue
    end
    if x == 'k'
        disp('Goodbye.')
        break; 
    end
    
end