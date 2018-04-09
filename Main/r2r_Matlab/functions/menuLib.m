function options = menuLib(steplog)

options = {};
switch steplog
    case 'a'
        disp('Main menu:')
        disp('c: Calibration          i: Idle')
        disp('r: Rotation             t: PID tuning')
        disp('d: Position control     f: Force control')
        disp('m: Memorizing           q: Quit')
        disp('')
        disp('1: Demo 1               2: Demo 2')
        disp('3: Demo 3               4: Demo 4')
        options = {'t', 'q'};
    case 'q'
        disp('Do not forget to rename the imported csv files')
        disp('      before running this program again!')
        disp('                   Goodbye!')      
    case 'at'
        disp('Which joint(s) are you going to control?')
        disp('1: Joint 1     2: Joint 2       3: Both') 
        disp('r: Return      a: Main menu     q: Quit')
        options = {'1', 'r', 'a', 'q'};  
    case 'at1'
        disp('What trajectory will be used?')
        disp('d: Default (f(t)=eta(1) rad)')
        disp('c: Costumed (referring to desirePIDtrajectory(t))')
        disp('r: Return      a: Main menu     q: Quit')
        options = {'d', 'r', 'a', 'q'};
    case 'at1d'
        disp('Enter the PID control gain as "[Kp, Ki, Kd]"')
        disp('o: Parameters of last time')
        disp('r: Return      a: Main menu     q: Quit')
        options = {-1};
    case 'at1dX'
        disp('Begin to run? y/n')
        disp('r: Return      a: Main menu     q: Quit')
        options = {'y', 'r', 'a', 'q'};
    case 'at1dXy'
        disp('Plot and save? y/n')
        disp('r: Redo/Return      a: Main menu     q: Quit')
        options = {'y', 'n', 'r', 'a', 'q'};
    case {'at1dXyy', 'at1dXyn'}
        disp('Try other parameters? y/n')
        disp('a: Main menu     q: Quit')
        options = {'y', 'n', 'r', 'a', 'q'};
end
end
%disp('System has been reset!')