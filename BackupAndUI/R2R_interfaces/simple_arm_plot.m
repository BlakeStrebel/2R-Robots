function simple_arm_plot(fig, jointangles)
%This is the function plotting the arms as a simulation by given the figure
%handle and a array of joint angles.

%***Choose the figure.
hold(fig, 'on')

%***Plot background.
plot(fig, 0, 0, 'Marker', '.', 'MarkerSize', 30, 'MarkerEdgeColor', 'k')

line(fig, [0, 1], [0, 0], 'LineStyle', '--', 'Color', 'k', 'LineWidth', 3)

%***Plot the moving parts.
dt = 0.05;
P = [];
format shortg
temp = clock();
t = temp(6);
i = 0;
% The simulation will delay dt second to begin.
while i < size(jointangles, 1)
    temp = clock();
    if temp(6) > t + dt
        t = temp(6);
        i = i + 1;
        delete(P);
        P = [line(fig, [0, cos(jointangles(i, 1))], [0, sin(jointangles(i, 1))], 'Color', 'k', 'LineWidth', 3);
             line(fig, [cos(jointangles(i, 1)), cos(jointangles(i, 1)) + cos(jointangles(i, 1) + jointangles(i, 2))], [sin(jointangles(i, 1)), sin(jointangles(i, 1)) + sin(jointangles(i, 1) + jointangles(i, 2))], 'Color', 'k', 'LineWidth', 3);
             line(fig, [cos(jointangles(i, 1)), 2 * cos(jointangles(i, 1))], [sin(jointangles(i, 1)), 2 * sin(jointangles(i, 1))], 'LineStyle', '--', 'Color', 'k', 'LineWidth', 3);
             plot(fig, cos(jointangles(i, 1)), sin(jointangles(i, 1)), 'Marker', '.', 'MarkerSize', 30, 'MarkerEdgeColor', 'k')]; 
         drawnow
    end
end