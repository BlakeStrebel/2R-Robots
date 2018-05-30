function menu = hierarchicalMenu(serial)
% This is the menu for all the hierarchical menu system. All submenus are
% treated as a node in a tree. All the processes and information are saved
% in the format of struct.

% Author: Huan Weng
% R2R team 
% May, 2018

% Template
id = 1;
menu(id).symbol = '';
menu(id).name = '';
menu(id).parent = 0;
menu(id).titledata = false;
menu(id).save = @(state, data) state;
menu(id).process = @(state) state;
menu(id).backup = [];
menu(id).title = '';
menu(id).titlelength = 0;
menu(id).question = '';
menu(id).children = [];
menu(id).optionslength = 0;
menu(id).data = false;

% Main menu
id = 2;
menu(id) = menu(1);
menu(id).symbol = 'a';
menu(id).title = ['                               R2R Arm Interface\n', ...
                ones(1, 80) * '-', '\n'];
menu(id).question = 'Main menu\n';
menu(id).children = [3, 5, 27, 62, 72, 73, 74, 75];

% Main menu -> Calibration
id = 3;
menu(id) = menu(1);
menu(id).symbol = 'c';
menu(id).name = 'Calibration';
menu(id).parent = 2;
menu(id).title = ['                                  Calibration\n  ', ...                                                  
                ones(1, 76) * '-', '\n'];
menu(id).question = 'Please move and hold the joints at their zero positions.\n';
menu(id).children = 4;

% Main menu -> Calibration -> Confirm
id = 4;
menu(id) = menu(1);
menu(id).symbol = 'c';
menu(id).name = 'Confirm and start';
menu(id).parent = 3;
menu(id).process = @(state) setZero(serial);
menu(id).title = ['                                  Calibrating\n    ', ...  
                ones(1, 72) * '-', '\n'];
menu(id).question = 'Calibration is finished.\n';

% Main menu -> Position reading
id = 5;
menu(id) = menu(3);
menu(id).symbol = 'p';
menu(id).name = 'Position reading';
menu(id).title = ['                                Position reading\n  ', ...                                                   
                ones(1, 76) * '-', '\n'];
menu(id).question = 'Choose joint(s) to be read:\n';
menu(id).children = [6, 13, 20];

% Main menu -> Position reading -> Motor 1
id = 6;
menu(id) = menu(1);
menu(id).symbol = '1';
menu(id).name = 'Motor 1';
menu(id).parent = 5;
menu(id).title = ['                                    Motor 1\n    ', ...                                                         
                  ones(1, 72) * '-', '\n'];
menu(id).question = 'Choose reading mode:\n';
menu(id).children = [7, 9];

% Main menu -> Position reading -> Motor 1 -> One time reading
id = 7;
menu(id) = menu(1);
menu(id).symbol = '1';
menu(id).name = 'One time reading';
menu(id).parent = 6;
menu(id).title = ['                                One time reading\n      ', ...                                                      
                ones(1, 68) * '-', '\n'];
menu(id).children = 8;

% Main menu -> Position reading -> Motor 1 -> One time reading -> Start
% reading
id = 8;
menu(id) = menu(1);
menu(id).symbol = 's';
menu(id).name = 'Start reading';
menu(id).parent = 7;
menu(id).titledata = true;
menu(id).process = @(state) positionRead(1, serial);
menu(id).save = @(state, data) positionSave(state, data, 1);
menu(id).title = ['            Motor 1 position: %9.4f rad(s), %9.4f degree(s)\n        ', ...   
                ones(1, 64) * '-', '\n'];

% Main menu -> Position reading -> Motor 1 -> Continuous reading
id = 9;
menu(id) = menu(1);
menu(id).symbol = 'c';
menu(id).name = 'Continuous reading';
menu(id).parent = 6;
menu(id).title = ['                               Continuous reading\n      ', ...                                                   
                ones(1, 68) * '-', '\n'];
menu(id).question = 'Set reading time (precision: 0.001s):\n';
menu(id).children = 10;
menu(id).data = true;

% Main menu -> Position reading -> Motor 1 -> Continuous reading -> Reading
% time
id = 10;
menu(id) = menu(1);
menu(id).parent = 9;
menu(id).titledata = true;
menu(id).save = @timeSave;
menu(id).process = @(state) timeSend(state.t, serial);
menu(id).backup = @timeGet;
menu(id).title = ['                            Reading time %9.4f s\n        ', ...    
                ones(1, 64) * '-', '\n'];
menu(id).children = 11;

% Main menu -> Position reading -> Motor 1 -> Continuous reading -> Reading
% time -> Start reading
id = 11;
menu(id) = menu(1);
menu(id).symbol = 's';
menu(id).name = 'Start reading';
menu(id).parent = 10;
menu(id).save = @(state, data) trajectorySave(state, data, 1);
menu(id).process = @(state) trajectoryPlot(state.t, 1, serial);
menu(id).title = ['                    Motor 1 position plotted in the figure\n          ', ...    
                ones(1, 60) * '-', '\n'];
menu(id).children = 12;

% Main menu -> Position reading -> Motor 1 -> Continuous reading -> Reading
% time -> Start reading -> Export trajectory
id = 12;
menu(id) = menu(1);
menu(id).symbol = 'e';
menu(id).name = 'Export trajectory';
menu(id).parent = 11;
menu(id).process = @(state) trajectoryExport(state.traj(:, 1));
menu(id).title = ['                           Motor 1 trajectory saved\n            ', ...                                                
                ones(1, 56) * '-', '\n'];
            
% Main menu -> Position reading -> Motor 2
id = 13;
menu(id) = menu(6);
menu(id).symbol = '2';
menu(id).name = 'Motor 2';
menu(id).title = ['                                    Motor 2\n    ', ...                                                         
                  ones(1, 72) * '-', '\n'];
menu(id).children = [14, 16];

% Main menu -> Position reading -> Motor 2 -> One time reading
id = 14;
menu(id) = menu(7);
menu(id).parent = 13;
menu(id).children = 15;

% Main menu -> Position reading -> Motor 2 -> One time reading -> Start
% reading
id = 15;
menu(id) = menu(8);
menu(id).parent = 14;
menu(id).process = @(state) positionRead(2, serial);
menu(id).save = @(state, data) positionSave(state, data, 2);
menu(id).title = ['            Motor 2 position: %9.4f rad(s), %9.4f degree(s)\n        ', ...   
                ones(1, 64) * '-', '\n'];

% Main menu -> Position reading -> Motor 2 -> Continuous reading
id = 16;
menu(id) = menu(9);
menu(id).parent = 13;
menu(id).children = 17;

% Main menu -> Position reading -> Motor 2 -> Continuous reading -> Reading
% time
id = 17;
menu(id) = menu(10);
menu(id).parent = 16;
menu(id).children = 18;

% Main menu -> Position reading -> Motor 2 -> Continuous reading -> Reading
% time -> Start reading
id = 18;
menu(id) = menu(11);
menu(id).parent = 17;
menu(id).save = @(state, data) trajectorySave(state, data, 2);
menu(id).process = @(state) trajectoryPlot(state.t, 2, serial);
menu(id).title = ['                    Motor 2 position plotted in the figure\n          ', ...    
                ones(1, 60) * '-', '\n'];
menu(id).children = 19;

% Main menu -> Position reading -> Motor 2 -> Continuous reading -> Reading
% time -> Start reading -> Export trajectory
id = 19;
menu(id) = menu(12);
menu(id).parent = 18;
menu(id).process = @(state) trajectoryExport(state.traj(:, 2));
menu(id).title = ['                           Motor 2 trajectory saved\n            ', ...                                                
                ones(1, 56) * '-', '\n'];
               
% Main menu -> Position reading -> Both motors
id = 20;
menu(id) = menu(6);
menu(id).symbol = 'b';
menu(id).name = 'Both motors';
menu(id).title = ['                                  Both motors\n    ', ...                                                     
                ones(1, 72) * '-', '\n'];
menu(id).children = [21, 23];

% Main menu -> Position reading -> Both motors -> One time reading
id = 21;
menu(id) = menu(7);
menu(id).parent = 20;
menu(id).children = 22;

% Main menu -> Position reading -> Both motors -> One time reading -> Start
% reading
id = 22;
menu(id) = menu(8);
menu(id).parent = 21;
menu(id).process = @(state) positionRead(3, serial);
menu(id).save = @(state, data) positionSave(state, data, 3);
menu(id).title = ['            Motor 1 position: %9.4f rad(s), %9.4f degree(s)\n', ...
                  '            Motor 2 position: %9.4f rad(s), %9.4f degree(s)\n        ', ...
                  ones(1, 64) * '-', '\n'];

% Main menu -> Position reading -> Both motors -> Continuous reading
id = 23;
menu(id) = menu(9);
menu(id).parent = 20;
menu(id).children = 24;

% Main menu -> Position reading -> Both motors -> Continuous reading ->
% Reading time
id = 24;
menu(id) = menu(10);
menu(id).parent = 23;
menu(id).children = 25;

% Main menu -> Position reading -> Both motors -> Continuous reading -> 
% Reading time -> Start reading
id = 25;
menu(id) = menu(11);
menu(id).parent = 24;
menu(id).save = @(state, data) trajectorySave(state, data, 3);
menu(id).process = @(state) trajectoryPlot(state.t, 3, serial);
menu(id).title = ['                  Both motor positions plotted in the figure\n          ', ...                                       
                ones(1, 60) * '-', '\n'];
menu(id).children = 26;

% Main menu -> Position reading -> Both motors -> Continuous reading -> 
% Reading time -> Start reading -> Export trajectory
id = 26;
menu(id) = menu(12);
menu(id).parent = 25;
menu(id).process = @(state) trajectoryExport(state.traj);
menu(id).title = ['                        Both motor trajectories saved\n            ', ...                                           
                ones(1, 56) * '-', '\n'];
            
% Main menu -> PID tuning
id = 27;
menu(id) = menu(1);
menu(id).symbol = 'i';
menu(id).name = 'PID tuning';
menu(id).parent = 2;
menu(id).title = ['                                  PID tuning\n  ', ...                                                     
                ones(1, 76) * '-', '\n'];
menu(id).question = ['Set initial positions in the format of [p1, p2] in rad(s)\n', ...
                     '(pi and expressions allowed):\n'];
menu(id).children = 28;
menu(id).data = true;

% Main menu -> PID tuning -> Position inputs
id = 28;
menu(id) = menu(1);
menu(id).parent = 27;
menu(id).titledata = true;
menu(id).save = @(state, data) positionSave(state, [data(1), 0 , data(2), 0], 3);
menu(id).backup = @(state) positionGet(state, 3);
menu(id).title = ['                   Motor 1 initial position: %9.4f rad(s)\n', ... 
                  '                   Motor 2 initial position: %9.4f rad(s)\n    ', ...
                ones(1, 72) * '-', '\n'];
menu(id).question = 'Choose joint(s) to be controlled:\n';
menu(id).children = [29, 40, 51];

% Main menu -> PID tuning -> Position inputs -> Motor 1
id = 29;
menu(id) = menu(6);
menu(id).parent = 28;
menu(id).title = ['               Motor 1 will be controlled, Motor 2 will be braked\n      ', ...  
                  ones(1, 68) * '-', '\n'];
menu(id).question = 'Set desired trajectory:\n';
menu(id).children = [30, 35];

% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Default
% trajectory
id = 30;
menu(id) = menu(1);
menu(id).symbol = 'd';
menu(id).name = 'Default desired trajectory (step)';
menu(id).parent = 29;
menu(id).save = @(state, data) trajectoryDefault(state);
menu(id).process = @(state) trajectorySend(state, 1);
menu(id).title = ['               Default desire trajectory (pi / 3 * varepsilon(t))\n        ', ...     
                  ones(1, 64) * '-', '\n'];
menu(id).question = 'Set PID gains in the format of [Kp, Ki, Kd]:\n';
menu(id).children = 31;
menu(id).data = true;

% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Default
% trajectory -> PID gain
id = 31;
menu(id) = menu(1);
menu(id).parent = 30;
menu(id).titledata = true;
menu(id).save = @(state, data) PIDSave(state, data, 1);
menu(id).process = @(state) PIDSend(state, 1);
menu(id).backup = @(state) PIDGet(state, 1);
menu(id).title = ['                               Motor 1 PID gain: \n', ...
                  '                 Kp = %9.4f, Ki = %9.4f, Kd = %9.4f\n          ', ...  
                  ones(1, 60) * '-', '\n'];
menu(id).children = 32;

% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Default
% trajectory -> PID gain -> Start running
id = 32;
menu(id) = menu(1);
menu(id).symbol = 's';
menu(id).name = 'Start running';
menu(id).parent = 31;
menu(id).save = @(state, data) trajectorySave(state, data, 1);
menu(id).process = @(state) trajectoryTracking(state, 1);
menu(id).title = ['                    Motor 1 trajectory plotted in the figure\n            ', ...  
                  ones(1, 56) * '-', '\n'];
menu(id).children = [33, 34];

% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Default
% trajectory -> PID gain -> Start running -> Export trajectory with desired
id = 33;
menu(id) = menu(12);
menu(id).symbol = 'w';
menu(id).name = 'Export trajectory with desired';
menu(id).parent = 32;
menu(id).process = @(state) trajectoryExport([state.traj(:, 1), ...
                                              state.dtraj(:, 1)]);
menu(id).title = ['               Desired and actual trajecotories of Motor 1 saved\n              ', ...                                                
                ones(1, 52) * '-', '\n'];         
            
% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Default
% trajectory -> PID gain -> Start running -> Export trajectory without
% desired
id = 34;
menu(id) = menu(12);
menu(id).symbol = 'o';
menu(id).name = 'Export trajectory without desired';
menu(id).parent = 32;
menu(id).process = @(state) trajectoryExport(state.traj(:, 1));
menu(id).title = ['                        Motor 1 actual trajectory saved\n              ', ...                                                
                ones(1, 52) * '-', '\n'];
            
% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Costumed
% trajectory
id = 35;
menu(id) = menu(30);
menu(id).symbol = 'c';
menu(id).name = 'Costumed trajectory';
menu(id).save = @(state, data) trajectoryCostumed(state);
menu(id).title = ['                           Costumed desire trajectory\n        ', ...    
                  ones(1, 64) * '-', '\n'];
menu(id).children = 36;

% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Costumed
% trajectory -> PID gain
id = 36;
menu(id) = menu(31);
menu(id).parent = 35;
menu(id).children = 37;

% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Costumed
% trajectory -> PID gain -> Start running
id = 37;
menu(id) = menu(32);
menu(id).parent = 36;
menu(id).children = [38, 39];

% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Costumed
% trajectory -> PID gain -> Start running -> Export trajectory with desired
id = 38;
menu(id) = menu(33);
menu(id).parent = 37;        
            
% Main menu -> PID tuning -> Position inputs -> Motor 1 -> Costumed
% trajectory -> PID gain -> Start running -> Export trajectory without
% desired
id = 39;
menu(id) = menu(34);
menu(id).symbol = 'o';
menu(id).parent = 37;    

% Main menu -> PID tuning -> Position inputs -> Motor 2
id = 40;
menu(id) = menu(13);
menu(id).parent = 28;
menu(id).title = ['               Motor 2 will be controlled, Motor 1 will be braked\n      ', ...  
                  ones(1, 68) * '-', '\n'];
menu(id).question = 'Set desired trajectory:\n';
menu(id).children = [41, 46];

% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Default
% trajectory
id = 41;
menu(id) = menu(30);
menu(id).parent = 40;
menu(id).process = @(state) trajectorySend(state, 2);
menu(id).children = 42;

% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Default
% trajectory -> PID gain
id = 42;
menu(id) = menu(31);
menu(id).parent = 41;
menu(id).save = @(state, data) PIDSave(state, data, 2);
menu(id).process = @(state) PIDSend(state, 2);
menu(id).backup = @(state) PIDGet(state, 2);
menu(id).title = ['                               Motor 2 PID gain: \n', ...
                  '                 Kp = %9.4f, Ki = %9.4f, Kd = %9.4f\n          ', ...  
                  ones(1, 60) * '-', '\n'];
menu(id).children = 43;

% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Default
% trajectory -> PID gain -> Start running
id = 43;
menu(id) = menu(32);
menu(id).parent = 42;
menu(id).save = @(state, data) trajectorySave(state, data, 2);
menu(id).process = @(state) trajectoryTracking(state, 2);
menu(id).title = ['                    Motor 2 trajectory plotted in the figure\n            ', ...  
                  ones(1, 56) * '-', '\n'];
menu(id).children = [44, 45];

% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Default
% trajectory -> PID gain -> Start running -> Export trajectory with desired
id = 44;
menu(id) = menu(33);
menu(id).parent = 43;
menu(id).process = @(state) trajectoryExport([state.traj(:, 2), state.dtraj(:, 2)]);
menu(id).title = ['               Desired and actual trajecotories of Motor 2 saved\n              ', ...                                                
                ones(1, 52) * '-', '\n'];         
            
% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Default
% trajectory -> PID gain -> Start running -> Export trajectory without
% desired
id = 45;
menu(id) = menu(34);
menu(id).parent = 43;
menu(id).process = @(state) trajectoryExport(state.traj(:, 2));
menu(id).title = ['                        Motor 2 actual trajectory saved\n              ', ...                                                
                ones(1, 52) * '-', '\n'];
            
% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Costumed
% trajectory
id = 46;
menu(id) = menu(35);
menu(id).children = 47;

% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Costumed
% trajectory -> PID gain
id = 47;
menu(id) = menu(42);
menu(id).parent = 46;
menu(id).children = 48;

% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Costumed
% trajectory -> PID gain -> Start running
id = 48;
menu(id) = menu(43);
menu(id).parent = 47;
menu(id).children = [49, 50];

% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Costumed
% trajectory -> PID gain -> Start running -> Export trajectory with desired
id = 49;
menu(id) = menu(44);
menu(id).parent = 48;        
            
% Main menu -> PID tuning -> Position inputs -> Motor 2 -> Costumed
% trajectory -> PID gain -> Start running -> Export trajectory without
% desired
id = 50;
menu(id) = menu(45);
menu(id).parent = 48;  

% Main menu -> PID tuning -> Position inputs -> Both motors
id = 51;
menu(id) = menu(20);
menu(id).parent = 28;
menu(id).title = ['                         Both motors will be controlled\n      ', ...  
                  ones(1, 68) * '-', '\n'];
menu(id).question = 'Set desired trajectory:\n';
menu(id).children = [52, 57];

% Main menu -> PID tuning -> Position inputs -> Both motors -> Default
% trajectory
id = 52;
menu(id) = menu(30);
menu(id).parent = 51;
menu(id).process = @(state) trajectorySend(state, 3);
menu(id).title = ['              Default desire trajectories (pi / 3 * varepsilon(t))\n        ', ...     
                  ones(1, 64) * '-', '\n'];
menu(id).question = 'Set PID gains in the format of [Kp1, Ki1, Kd1, Kp2, Ki2, Kd2]:\n';
menu(id).children = 53;

% Main menu -> PID tuning -> Position inputs -> Both motors -> Default
% trajectory -> PID gain
id = 53;
menu(id) = menu(31);
menu(id).parent = 52;
menu(id).save = @(state, data) PIDSave(state, data, 3);
menu(id).process = @(state) PIDSend(state, 3);
menu(id).backup = @(state) PIDGet(state, 3);
menu(id).title = ['                               Motor 1 PID gain: \n', ...
                  '                 Kp = %9.4f, Ki = %9.4f, Kd = %9.4f\n', ...  
                  '                               Motor 2 PID gain: \n', ...
                  '                 Kp = %9.4f, Ki = %9.4f, Kd = %9.4f\n          ', ...
                  ones(1, 60) * '-', '\n'];
menu(id).children = 54;

% Main menu -> PID tuning -> Position inputs -> Both motors -> Default
% trajectory -> PID gain -> Start running
id = 54;
menu(id) = menu(32);
menu(id).parent = 53;
menu(id).save = @(state, data) trajectorySave(state, data, 3);
menu(id).process = @(state) trajectoryTracking(state, 3);
menu(id).title = ['                 Both motor trajectories plotted in the figure\n            ', ...            
                  ones(1, 56) * '-', '\n'];
menu(id).children = [55, 56];

% Main menu -> PID tuning -> Position inputs -> Both motors -> Default
% trajectory -> PID gain -> Start running -> Export trajectory with desired
id = 55;
menu(id) = menu(33);
menu(id).parent = 54;
menu(id).process = @(state) trajectoryExport([state.traj, state.dtraj]);
menu(id).title = ['             Desired and actual trajecotories of both motors saved\n              ', ...                                                
                ones(1, 52) * '-', '\n'];         
            
% Main menu -> PID tuning -> Position inputs -> Both motors -> Default
% trajectory -> PID gain -> Start running -> Export trajectory without
% desired
id = 56;
menu(id) = menu(34);
menu(id).parent = 54;
menu(id).process = @(state) trajectoryExport(state.traj);
menu(id).title = ['                      Both motor actual trajectories saved\n              ', ...                                                
                ones(1, 52) * '-', '\n'];
            
% Main menu -> PID tuning -> Position inputs -> Both motors -> Costumed
% trajectory
id = 57;
menu(id) = menu(52);
menu(id).parent = 51;
menu(id).symbol = 'c';
menu(id).name = 'Costumed trajectories';
menu(id).save = @(state, data) trajectoryCostumed(state);
menu(id).title = ['                          Costumed desire trajectories\n        ', ...    
                  ones(1, 64) * '-', '\n'];
menu(id).children = 58;

% Main menu -> PID tuning -> Position inputs -> Both motors -> Costumed
% trajectory -> PID gain
id = 58;
menu(id) = menu(53);
menu(id).parent = 57;
menu(id).children = 59;

% Main menu -> PID tuning -> Position inputs -> Both motors -> Costumed
% trajectory -> PID gain -> Start running
id = 59;
menu(id) = menu(54);
menu(id).parent = 58;
menu(id).children = [60, 61];

% Main menu -> PID tuning -> Position inputs -> Both motors -> Costumed
% trajectory -> PID gain -> Start running -> Export trajectory with desired
id = 60;
menu(id) = menu(55);
menu(id).parent = 59;   
            
% Main menu -> PID tuninga -> Position inputs -> Both motors -> Costumed
% trajectory -> PID gain -> Start running -> Export trajectory without
% desired
id = 61;
menu(id) = menu(56);
menu(id).parent = 59;  

% Main menu -> Rotation
id = 62;
menu(id) = menu(5);
menu(id).symbol = 'v';
menu(id).name = 'Rotation';
menu(id).title = ['                                    Rotation\n  ', ...                                  
                 ones(1, 76) * '-', '\n'];
menu(id).children = [63, 64, 65];

% Main menu -> Rotation -> Motor 1
id = 63;
menu(id) = menu(6);
menu(id).parent = 62;
menu(id).title = ['               Motor 1 will be controlled, Motor 2 will be braked\n    ', ...  
                  ones(1, 72) * '-', '\n'];
menu(id).question = 'Set velocity as percentage (%%), positive for cw and negative for ccw:\n';
menu(id).children = 66;
menu(id).data = true;

% Main menu -> Rotation -> Motor 2
id = 64;
menu(id) = menu(13);
menu(id).parent = 62;
menu(id).title = ['               Motor 2 will be controlled, Motor 1 will be braked\n    ', ...  
                  ones(1, 72) * '-', '\n'];
menu(id).question = 'Set velocity as percentage (%%), positive for cw and negative for ccw:\n';
menu(id).children = [];
menu(id).data = true;

% Main menu -> Rotation -> Both motors
id = 65;
menu(id) = menu(20);
menu(id).parent = 62;
menu(id).title = ['                         Both motors will be controlled\n    ', ...  
                  ones(1, 72) * '-', '\n'];
menu(id).question = 'Set velocities as percentages in the form of [p1, p2]:\n';
menu(id).children = [];
menu(id).data = true;

% Main menu -> Rotation -> Motor 1 -> Velocity inputs
id = 66;
menu(id) = menu(1);
menu(id).parent = 63;
menu(id).titledata = true;
menu(id).save = @(state, data) velocitySave(state, data, 1);
menu(id).backup = @(state) velocityGet(state, 1);
menu(id).title = ['                       Motor 1 rotation speed: %9.4f %%\n      ', ... 
                  ones(1, 68) * '-', '\n'];
menu(id).children = 69;

% Main menu -> Rotation -> Motor 2 -> Velocity inputs
id = 67;
menu(id) = menu(1);
menu(id).parent = 64;
menu(id).titledata = true;
menu(id).save = @(state, data) velocitySave(state, data, 2);
menu(id).backup = @(state) velocityGet(state, 2);
menu(id).title = ['                       Motor 2 rotation speed: %9.4f %%\n      ', ... 
                  ones(1, 68) * '-', '\n'];
menu(id).children = 70;

% Main menu -> Rotation -> Both motors -> Velocity inputs
id = 68;
menu(id) = menu(1);
menu(id).parent = 65;
menu(id).titledata = true;
menu(id).save = @(state, data) velocitySave(state, data, 3);
menu(id).backup = @(state) velocityGet(state, 3);
menu(id).title = ['                       Motor 1 rotation speed: %9.4f %%\n', ... 
                  '                       Motor 2 rotation speed: %9.4f %%\n      ', ... 
                  ones(1, 68) * '-', '\n'];
menu(id).children = 71;

% Main menu -> Rotation -> Motor 1 -> Velocity inputs -> Start running
id = 69;
menu(id) = menu(1);
menu(id).symbol = 's';
menu(id).name = 'Start rotating';
menu(id).parent = 66;
menu(id).process = @(state) velocitySend(state.v, 1, serial);
menu(id).title = ['                              Motor 1 is rotating\n        ', ...   
                ones(1, 64) * '-', '\n'];

% Main menu -> Rotation -> Motor 2 -> Velocity inputs -> Start running
id = 70;
menu(id) = menu(1);
menu(id).symbol = 's';
menu(id).name = 'Start rotating';
menu(id).parent = 67;
menu(id).process = @(state) velocitySend(state.v, 2, serial);
menu(id).title = ['                              Motor 2 is rotating\n        ', ...   
                ones(1, 64) * '-', '\n'];
            
% Main menu -> Rotation -> Both motors -> Velocity inputs -> Start running
id = 71;
menu(id) = menu(1);
menu(id).symbol = 's';
menu(id).name = 'Start rotating';
menu(id).parent = 68;
menu(id).process = @(state) velocitySend(state.v, 3, serial);
menu(id).title = ['                            Both motor are rotating\n        ', ...   
                ones(1, 64) * '-', '\n'];
            
% Main menu -> Settings
id = 72;
menu(id) = menu(5);
menu(id).symbol = 's';
menu(id).name = 'Settings  (developing)';
menu(id).title = ['                                    Settings\n  ', ...                                  
                 ones(1, 76) * '-', '\n'];
menu(id).children = [];

% Main menu -> Temp reading
id = 73;
menu(id) = menu(5);
menu(id).symbol = 't';
menu(id).name = 'Temp reading (developing)';
menu(id).title = ['                                  Temp reading\n  ', ...                                                    
                 ones(1, 76) * '-', '\n'];
menu(id).children = [];

% Main menu -> Demo 1
id = 74;
menu(id) = menu(5);
menu(id).symbol = '1';
menu(id).name = 'Demo 1 (developing)';
menu(id).title = ['                                     Demo 1\n  ', ...                                                     
                 ones(1, 76) * '-', '\n'];
menu(id).children = [];

% Main menu -> Demo 2
id = 75;
menu(id) = menu(74);
menu(id).symbol = '2';
menu(id).name = 'Demo 2 (developing)';
menu(id).title = ['                                     Demo 2\n  ', ...                                                     
                 ones(1, 76) * '-', '\n'];
menu(id).children = [];


            
%{
% Main menu -> Position reading
id = 100;
menu(id) = menu(3);
menu(id).symbol = 'p';
menu(id).name = 'Position reading';
menu(id).title = ['                                Position reading\n  ', ...                                                   
                ones(1, 76) * '-', '\n'];
menu(id).question = 'Choose joint(s) to be read:\n';
menu(id).children = [6, 13, 20];







% Main menu -> Position reading -> Motor 1
id = 6;
menu(id) = menu(1);
menu(id).symbol = '1';
menu(id).name = 'Motor 1';
menu(id).parent = 5;
menu(id).title = ['                                        r 1\n    ', ...                                                         
                ones(1, 72) * '-', '\n'];
menu(id).question = 'Choose reading mode:\n';
menu(id).children = [7, 9];







%}


%{
menu(1).title = ['                                        nterface \n', ... 
                ones(1, 80) * '-', '\n'];
%}
end