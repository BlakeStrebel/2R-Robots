function successInfo(steplog, i, data)

switch steplog(1: i + 1)
    case 'at'
        disp('Mode: PID tuning')
    case 'at1'
        disp('Controlled joint(s): Joint 1')
        disp('Joint 2 will keep its position')
    case 'at1d'
        disp('Default trajectory applied')
    case 'at1dX'
        disp('PID parameters set as')
        disp(['Kp = ', num2str(data(1)), ', Ki = ', num2str(data(2)), ', Kd = ', num2str(data(3))])
    case 'at1dXy'
        disp('Complete')    
    case 'at1dXyy'
        disp('Plotted and saved')       
    case 'at2'
        disp('Controlled joint(s): Joint 2')
        disp('Joint 1 will keep its position')
    case 'at3'
        disp('Controlled joint(s): Both joints')
end
%{
modesymbol = ['Mode', 'a', 'Main menu';
              'x', 'c', 'Calibration';
              'x', 'i', 'Idle';
              'x', 's', 'Rotation';
              'x', 'p', 'PID tuning';
              'x', 'd', 'Position control';
              'x', 'f', 'Force control';
              'x', 'm', 'Memorizing';
              'x', 'q', 'Quit';
              'x', '1', 'Demo 1';
              'x', '2', 'Demo 2';
              'x', '3', 'Demo 3';
              'x', '4', 'Demo 4']
%}
        