function data = trajectorySend(traj, motor, serial)
if motor == 1
    fwrite(serial, 14);
    n = length(traj(:, 1));
    fwrite(serial, n, 'int');
    for i = 1: n
        fwrite(serial, traj(i, 1), 'float32');
    end
elseif motor == 2
    fwrite(serial, 15);
    n = length(traj(:, 2));
    fwrite(serial, n, 'int');
    for i = 1: n
        fwrite(serial, traj(i, 2), 'float32');
    end
else
    fwrite(serial, 16);
    n = size(traj, 1) * 2;
    list = reshape(traj, n, 1);  
    fwrite(serial, n, 'int');
    for i = 1: n
        fwrite(serial, list(i), 'float32');
    end
end
data = traj;
end