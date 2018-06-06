function [stepnew, statenew, menunew] ...
         = commandOperation(stepold, stateold, response, menuold, serial)
% This function proceeds all the response, print the menu and save the
% data. 

%*** Initialize the outputs.
stepnew = stepold; % Inherit all values.
statenew = stateold; % Inherit all values.
menunew = menuold; % Inherit all values.
data = []; % Initialize variable.
%*** Delete response line.
fprintf(repmat('\b', 1, length(response) + 17));
%*** Deal with Quit.
if response == 'q'
    robotRelax(serial);
    return % Return directly.
%*** Deal with Main menu.
elseif response == 'a' 
    robotRelax(serial);
    stepnew = 2; % Reset linked list.
    clc; % Clear all contents.
%*** Deal with Return.    
elseif length(response) == 1 && response == 'r' && stepold ~= 1
    robotRelax(serial);
    stepnew = menuold(stepold).parent; % Return back to parent.
    fprintf(repmat('\b', 1, menuold(stepold).optionslength ...
                            + menuold(stepold).titlelength ...
                            + menuold(stepnew).titlelength)); % Delete the submenu to reprint.
%*** Deal with number input. 
elseif menuold(stepold).data
    data = str2num(response); % Convert and save the values.
    if isempty(data)
        return % Return directly.
    else
        stepnew = menuold(stepold).children;
        fprintf(repmat('\b', 1, menuold(stepold).optionslength)); % Delete parent options.
    end
else
    optionlist = menuold(stepold).children; % Get children list.
    for i = 1: length(optionlist)
        if response == menuold(optionlist(i)).symbol
            stepnew = optionlist(i); % Convert input to menu id.
            fprintf(repmat('\b', 1, menuold(stepold).optionslength)); % Delete parent options.
        end
    end
    if stepnew == stepold
        return % Return directly.
    end
end
%*** Deal with main process, saving data and menu title.
if menunew(stepnew).titledata
    if ~isempty(data)
        statenew = menunew(stepnew).save(statenew, data); % Update state using the input.
        menunew(stepnew).titlelength = fprintf(menunew(stepnew).title, data); % Print menu title.
        menunew(stepnew).process(statenew); % Deal with the input.
    elseif response == 'r'
        data = menunew(stepnew).backup(statenew);
        menunew(stepnew).titlelength = fprintf(menunew(stepnew).title, data); % Print menu title.
    else
        data = menunew(stepnew).process(statenew); % Deal with the input.
        menunew(stepnew).titlelength = fprintf(menunew(stepnew).title, data); % Print menu title.
        statenew = menunew(stepnew).save(statenew, data); % Update state using the input.
    end    
else
    data = menunew(stepnew).process(statenew); % Deal with the input.
    if ~isempty(data)
        statenew = menunew(stepnew).save(statenew, data); % Update state using the input.
    end
    menunew(stepnew).titlelength = fprintf(menunew(stepnew).title); % Print menu title without values.
end    
%*** Print current menu.
optionlist = menunew(stepnew).children; % Get children list.
if ~isempty(optionlist) && menunew(stepnew).data
    optionlist = optionlist(1: end - 1); % Get children list.
end
infolength = fprintf(menunew(stepnew).question); % Initialize the option length.
for i = 1: 2: length(optionlist) - 1
    fprintf('%-40s', [menunew(optionlist(i)).symbol, ': ', menunew(optionlist(i)).name]); % Print the left option.
    fprintf('%-40s\n', [menunew(optionlist(i + 1)).symbol, ': ', menunew(optionlist(i + 1)).name]); % Print the right option.
    infolength = infolength + 81; % Get option length.
end
if rem(length(optionlist), 2)
    fprintf('%-40s\n', [menunew(optionlist(end)).symbol, ': ',menunew(optionlist(end)).name]) % Print the last odd option.
    infolength = infolength + 41; % Get option length.
end
%*** Deal with additional options.
if stepnew == 1
    infolength = infolength + fprintf('q: Quit\n'); % Add Quit to main menu.
else
    infolength = infolength + fprintf('r: Return        a: Main menu        q: Quit\n'); % Add additional options.
end
menunew(stepnew).optionslength = infolength; % Record the total menu length.

%%*** Deal with other available responses.
%{     
msg      
     
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
         %}
end