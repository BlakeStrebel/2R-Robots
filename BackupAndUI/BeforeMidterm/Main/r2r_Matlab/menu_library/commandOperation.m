function [commandnew, statenew, textnew] ...
         = commandOperation(commandold, stateold, response, textold)
     
statenew = stateold;
commandnew = commandold;
textnew = textold;
same = 1;
data = [];

if commandold.options(1) == 'X'
	data = str2num(response);
	if ~isempty(data)
        commandnew.log = [commandold.log, 'X'];
        same = 0;
	end 
elseif length(response) == 1
    if response == 'a' || response == 'q'
        commandnew.log = response;
        same = 0;
    elseif ismember(response, commandold.options)
        commandnew.log = [commandold.log, response];  
        same = 0;
    end
end

if  length(response) == 1 && response == 'r' && commandold.options(end) == 'r'
	commandnew.log = commandold.log(1: end - 1);
    same = 0;
    fprintf(repmat('\b', 1, textold.menu(end) + textold.success(end) + ...
                           textold.success(end - 1) - textold.menu(end - 2)));                      
    textnew.menu = textold.menu(1: end - 2);
    textnew.success = textold.success(1: end - 2);
end

fprintf(repmat('\b', 1, length(response) + 17));

if same == 0
    switch commandnew.log
        case 'a'
            clc;
            welcomeTitle; 
            % doing t now. finished p.
            msg = ['Main menu:\n', ...
                   'c: Calibration          i: Idle\n', ...
                   'p: Position reading     r: Rotation\n', ...   
                   'm: Memorizing           t: PID tuning\n', ...
                   'd: Position control     f: Force control\n', ...
                   'q: Quit\n', ...
                   '1: Demo 1               2: Demo 2\n', ...
                   '3: Demo 3               4: Demo 4\n'];
            textnew.menu = [textnew.menu, fprintf(msg)]; 
            commandnew.options = 'qcpt';
        case 'at'
            fprintf(repmat('\b', 1, textnew.menu(end)));        
            msg = ['                                   PID tuning\n  ', ...                                               
                   ones(1, 76) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];   
            msg = ['Choose joint to be controlled:\n', ...
                  '1: Joint 1     2: Joint 2       b: Both\n', ...
                   'r: Return      a: Main menu     q: Quit\n'];    
            textnew.menu = [textnew.menu, fprintf(msg)]; 
            commandnew.options = 'aq12br';          
        case {'at1', 'at2', 'atb'}
            fprintf(repmat('\b', 1, textnew.menu(end)));
            if commandnew.log(end) == '1'
                msg = ['               Motor 1 will be controlled, Motor 2 will be braked\n    ', ...                        
                       ones(1, 72) * '-', '\n'];
            elseif commandnew.log(end) == '2'
                msg = ['               Motor 2 will be controlled, Motor 1 will be braked\n    ', ...  
                       ones(1, 72) * '-', '\n'];   
            else
                msg = ['                                  Both motors\n    ', ...
                       ones(1, 72) * '-', '\n'];
            end
            textnew.success = [textnew.success, fprintf(msg)];    
            msg = ['Set the trajectory in test', ...
                   'd: Default trajectory(s): Step function(s) for 10s', ...
                   'c: Costume function defined by function(s) trajGenerator1/trajGenerator2', ...
                   'r: Return        a: Main menu      q: Quit\n'];    
            textnew.menu = [textnew.menu, fprintf(msg)];
            commandnew.options = 'aqdr';            
        case {'at1d', 'at2d'}    
        	msg = ['                               Default trajectory\n      ', ...                                                      
                   ones(1, 68) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];            
            msg = ['Set the PID parameters in the format of [Kp, Ki, Kd]', ...
                   'd: Default PID parameters', ...
                   'r: Return        a: Main menu      q: Quit\n'];                 
            textnew.success = [textnew.success, fprintf(msg)];
            commandnew.options = 'Xaqr'; 
        case {'at1dX', 'at2dX'}    
        	msg = ['Kp = %9.4f, Kp = %9.4f, Kp = %9.4f\n        ', ... 
                   ones(1, 64) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg, data(1), data(2), data(3))];   
            if commandnew.log(3) == '1'
                statenew.Kp(1) = data(1);
                statenew.Ki(1) = data(2);
                statenew.Kd(1) = data(3);
            else
                statenew.Kp(2) = data(1);
                statenew.Ki(2) = data(2);
                statenew.Kd(2) = data(3);     
            end
            msg = ['Moving to the inital positions? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n'];                 
            textnew.success = [textnew.success, fprintf(msg)];
            commandnew.options = 'aqyr'; 
        case {'at1dXy', 'at2dXy'}    
        	msg = ['                                     Moving\n        ', ...      
                   ones(1, 60) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg, data(1), data(2), data(3))];   
            moveToInitial;
            msg = ['Start running? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n'];                 
            textnew.success = [textnew.success, fprintf(msg)];
            commandnew.options = 'aqyr';
        case {'at1dXyy', 'at2dXyy'}    
        	msg = ['                                    Running\n          ', ...      
                   ones(1, 56) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg, data(1), data(2), data(3))];   
            trajectoryTracking;
            msg = ['Start running? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n'];                 
            textnew.success = [textnew.success, fprintf(msg)];
            commandnew.options = 'aqyr';            
            if commandnew.log(3) == '1'
                msg = ['Motor 1 position (rad) being plotted in the figure\n', ...
                       'Save the data? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n']; 
                textnew.menu = [textnew.menu, fprintf(msg)];       
                statenew.traj1 = positionPlot(1, stateold.t);               
            else
                msg = ['Motor 2 position (rad) being plotted in the figure\n', ...
                       'Save the data? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n']; 
                textnew.menu = [textnew.menu, fprintf(msg)];       
                statenew.traj1 = positionPlot(2, stateold.t); 
            end
            commandnew.options = 'aqyr';  
                
                
                
        case 'ac'
            fprintf(repmat('\b', 1, textnew.menu(end)));        
            msg = ['                                  Calibration\n  ', ...  
                   ones(1, 76) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];   
            msg = ['Move and hold the joints to their zero positions.\n', ...
                   'Start calibrating? y/n\n', ...
                   'r: Return      a: Main menu     q: Quit\n'];    
            textnew.menu = [textnew.menu, fprintf(msg)]; 
            commandnew.options = 'aqyr';    
        case 'acy'
            fprintf(repmat('\b', 1, textnew.menu(end)));        
            msg = ['                                  Calibrating\n    ', ...  
                   ones(1, 72) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];   
            setZero;
            msg = ['Calibration finished.\n', ...
                   'r: Return      a: Main menu     q: Quit\n'];    
            textnew.menu = [textnew.menu, fprintf(msg)]; 
            commandnew.options = 'aqr';   
            
        case 'ap'
            fprintf(repmat('\b', 1, textnew.menu(end)));        
            msg = ['                                Position reading\n  ', ...
                   ones(1, 76) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];   
            msg = ['Choose joint to be read:\n', ...
                  '1: Joint 1     2: Joint 2       b: Both\n', ...
                   'r: Return      a: Main menu     q: Quit\n'];    
            textnew.menu = [textnew.menu, fprintf(msg)]; 
            commandnew.options = 'aq12br'; 
        case {'ap1', 'ap2', 'apb'}
            fprintf(repmat('\b', 1, textnew.menu(end)));
            if commandnew.log(end) == '1'
                msg = ['                                    Motor 1\n    ', ...
                       ones(1, 72) * '-', '\n'];
            elseif commandnew.log(end) == '2'
                msg = ['                                    Motor 2\n    ', ...
                       ones(1, 72) * '-', '\n'];   
            else
                msg = ['                                  Both motors\n    ', ...
                       ones(1, 72) * '-', '\n'];
            end
            textnew.success = [textnew.success, fprintf(msg)];    
            msg = ['Choose reading mode:\n', ...
                   '1: One time reading     c: Continuous plot\n', ...
                   'r: Return        a: Main menu      q: Quit\n'];    
            textnew.menu = [textnew.menu, fprintf(msg)];
            commandnew.options = 'aq1cr';        
        case {'ap11', 'ap21', 'apb1'}
            fprintf(repmat('\b', 1, textnew.menu(end)));
            msg = ['                                One time reading\n      ', ...
                   ones(1, 68) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];
            msg = ['Start reading? y/n\n', ...
                   'r: Return        a: Main menu      q: Quit\n'];
            textnew.menu = [textnew.menu, fprintf(msg)];   
            commandnew.options = 'aqyr';
        case {'ap11y', 'ap21y'}
            fprintf(repmat('\b', 1, textnew.menu(end)));
            msg = ['                                  Reading now\n        ', ...  
                   ones(1, 64) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];         
            if commandnew.log(3) == '1'
                msg = ['Motor 1 position: %9.4f rad(s), %9.4f degree(s)\n', ...
                       'r: Return        a: Main menu      q: Quit\n']; 
                data = positionRead(1);
            else
                msg = ['Motor 2 position: %9.4f rad(s), %9.4f degree(s)\n', ...
                       'r: Return        a: Main menu      q: Quit\n']; 
                data = positionRead(2);  
            end
            statenew.p1 = data;
            textnew.menu = [textnew.menu, fprintf(msg, data, data / pi * 180)]; 
            commandnew.options = 'aqr';
        case 'apb1y'
            fprintf(repmat('\b', 1, textnew.menu(end)));
            msg = ['                                  Reading now\n        ', ...                                  
                   ones(1, 64) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)]; 
            msg = ['Motor 1 position: %9.4f rad(s), %9.4f degree(s)\n', ...
                   'Motor 2 position: %9.4f rad(s), %9.4f degree(s)\n', ...
                   'r: Return        a: Main menu      q: Quit\n']; 
            data = positionRead(3);
            statenew.p1 = data(1);
            statenew.p2 = data(2);
            textnew.menu = [textnew.menu, fprintf(msg, data(1), data(1) / pi * 180, data(2), data(2) / pi * 180)]; 
            commandnew.options = 'aqr';            
        case {'ap1c', 'ap2c', 'apbc'}
            fprintf(repmat('\b', 1, textnew.menu(end)));
            msg = ['                               Continuous plot(s)\n      ', ...
                   ones(1, 68) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)]; 
            msg = ['Enter the duration in second (precision: 0.001)\n', ...
                   'r: Return        a: Main menu      q: Quit\n']; 
            textnew.menu = [textnew.menu, fprintf(msg)]; 
            commandnew.options = 'Xaqr';
        case {'ap1cX', 'ap2cX', 'apbcX'}
            fprintf(repmat('\b', 1, textnew.menu(end)));
            msg = ['                              %9.4f seconds\n        ', ...
                   ones(1, 64) * '-', '\n'];
            if isempty(data)
                textnew.success = [textnew.success, fprintf(msg, stateold.t)]; 
            else
                statenew.t = data;
                textnew.success = [textnew.success, fprintf(msg, data)]; 
            end
            msg = ['Start reading? y/n\n', ...
                   'r: Return        a: Main menu      q: Quit\n'];
            textnew.menu = [textnew.menu, fprintf(msg)];   
            commandnew.options = 'aqyr';            
        case {'ap1cXy', 'ap2cXy', 'apbcXy'}            
            fprintf(repmat('\b', 1, textnew.menu(end)));
            msg = ['                                  Reading now\n          ', ...                                  
                   ones(1, 60) * '-', '\n'];
            textnew.success = [textnew.success, fprintf(msg)];                               
            if commandnew.log(3) == '1'
                msg = ['Motor 1 position (rad) being plotted in the figure\n', ...
                       'Save the data? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n']; 
                textnew.menu = [textnew.menu, fprintf(msg)];       
                statenew.traj1 = positionPlot(1, stateold.t);
            elseif commandnew.log(3) == '2'
                msg = ['Motor 2 position (rad) being plotted in the figure\n', ...
                       'Save the data? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n']; 
                textnew.menu = [textnew.menu, fprintf(msg)];       
                statenew.traj1 = positionPlot(2, stateold.t);
            else 
                msg = ['Motors positions (rad) being plotted in the figure\n', ...
                       'Save the data? y/n\n', ...
                       'r: Return        a: Main menu      q: Quit\n']; 
                textnew.menu = [textnew.menu, fprintf(msg)];       
                data = positionPlot(3, stateold.t);
                statenew.traj1 = data(1, :);
                statenew.traj2 = data(2, :);              
            end
            commandnew.options = 'aqyr';
        case {'ap1cXyy', 'ap2cXyy', 'apbcXyy', 'at1dXyyy', 'at2dXyyy'}            
            fprintf(repmat('\b', 1, textnew.menu(end)));
            msg = ['                                Saving data now\n            ', ...                                                     
                   ones(1, 56) * '-', '\n'];            
            textnew.success = [textnew.success, fprintf(msg)];   
            date = datestr(now,'MM_dd_yyyy_HH_mm_ss');
            if commandnew.log(3) == '1'                
                csvwrite([date, '.csv'], stateold.traj1');
            elseif commandnew.log(3) == '2'
                csvwrite([date, '.csv'], stateold.traj2');
            else 
                csvwrite([date, '.csv'], [stateold.traj1; stateold.traj2]');
            end
            msg = ['Data saved in', date, '.csv\n', ...
                   'r: Return        a: Main menu      q: Quit\n']; 
            textnew.menu = [textnew.menu, fprintf(msg)];              
            commandnew.options = 'aqr';
    end
end
end