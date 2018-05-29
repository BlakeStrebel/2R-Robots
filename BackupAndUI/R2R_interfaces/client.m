addpath('menu_library', 'matlab_library');

clc; clear all;

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

welcomeTitle;
response = input('Enter the using port: ', 's');
clc;
userserial = serialConfig(response);
fopen(userserial);
finishup = onCleanup(@()fclose(userserial)); %closes serial on termination!

response = 'a';
r2rstate = struct('p', [0, 0], 'v', [0, 0], 'traj', [0, 0], ...
                  'dtraj', [0, 0], 'Kp', [1, 2], 'Ki', [1, 2], ...
                  'Kd', [1, 2], 't', 0);

r2rstate = defaultPID(r2rstate);
menu = hierarchicalMenu(userserial);
currentmenu = 2;

while true
    [currentmenu, r2rstate, menu] ...
    = commandOperation(currentmenu, r2rstate, response, menu, userserial);
    response = input('Enter value(s): ', 's');
    if response == 'q'
        break
    end
end
fairWell;