addpath('menu_library', 'matlab_library');

clc; clear all;
text = welcomeTitle;

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
command = struct('log', 'a', 'options', 'q');
text = struct('menu', text, 'success', text);

r2rstate = defaultPID(r2rstate);

while true
    [command, r2rstate, text] = commandOperation(command, r2rstate, ...
                                                 response, text);
    response = input('Enter value(s): ', 's');
    if response == 'q'
        break
    end
end
fairWell;
    