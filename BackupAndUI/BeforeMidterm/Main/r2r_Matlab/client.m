addpath('menu_library', 'matlab_library');

clc; clear all;

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

response = input('Enter the using port: ', 's');
userserial = serialConfig(response);
%fopen(userserial);
finishup = onCleanup(@()fclose(userserial)); %closes serial on termination!

response = 'a';
r2rstate = struct('p1', 0, 'p2', 0, 'v1', 0, 'v2', 0, 'traj1', [], ...
                  'traj2', [], 'dtraj1', [], 'dtraj2', [], 'Kp', [1, 2], ...
                  'Ki', [1, 2], 'Kd', [1, 2], 't', 0);
commandlog = 'a';

r2rstate = defaultPID(r2rstate);
menu = hierarchicalMenu;

while true
    [commandlog, r2rstate] = commandOperation(commandlog, r2rstate, ...
                                                 response);
    response = input('Enter value(s): ', 's');
    if response == 'q'
        break
    end
end
fairWell;