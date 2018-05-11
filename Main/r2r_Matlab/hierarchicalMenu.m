function menu = hierarchicalMenu()
% This is the menu for all the hierarchical menu system. All submenus are
% treated as a node in a tree. All the processes and information are saved
% in the format of struct.

% Author: Huan Weng
% R2R team 
% May, 2018

%%
% Main menu
menu(1).name = '';
menu(1).parent = 0;
menu(1).titleprocess = [];
menu(1).title = ['                               R2R Arm Interface \n', ...
                ones(1, 80) * '-', '\n'];
menu(1).process = [];
menu(1).question = 'Main menu';
menu(1).options = [];



end