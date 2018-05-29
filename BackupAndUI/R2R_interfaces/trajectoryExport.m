function success = trajectoryExport(state, motor, desire)
name = [datestr(now,'mm_dd_yyyy_HH_MM_ss'), '.csv'];
if desire == 'o'
    if motor == 1
        csvwrite(name, state.traj(:, 1));
    elseif motor == 2
        csvwrite(name, state.traj(:, 2));   
    else 
        csvwrite(name, state.traj);        
    end
else
    if motor == 1
        csvwrite(name, [state.traj(:, 1), state.dtraj(:, 1)]);
    elseif motor == 2
        csvwrite(name, [state.traj(:, 2), state.dtraj(:, 2)]);   
    else 
        csvwrite(name, [state.traj, state.dtraj]);      
    end   
end
success = [];
end