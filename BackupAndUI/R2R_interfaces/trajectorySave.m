function statenew = trajectorySave(stateold, data, motor, desired)
statenew = stateold;
if desired
    statenew.t = size(data, 1);
    statenew.dtraj = zeros(statenew.t, 2); 
    if motor == 1 
        statenew.dtraj(:, 1) = data(:, 1);
    elseif motor == 2
        statenew.dtraj(:, 2) = data(:, 2); 
    else
        statenew.dtraj = data; 
    end
else
    statenew.traj = zeros(statenew.t, 2); 
    if motor == 1 
        statenew.traj(:, 1) = data(:, 1);
    elseif motor == 2
        statenew.traj(:, 2) = data(:, 2); 
    else
        statenew.traj = data; 
    end        
end
end