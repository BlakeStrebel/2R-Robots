function statenew = trajectorySave(stateold, data, motor)
statenew = stateold;
statenew.traj = zeros(size(data, 1), 2);
if motor == 1 
    statenew.traj(:, 1) = data;
elseif motor == 2
    statenew.traj(:, 2) = data; 
else
    statenew.traj = data;  
end
end