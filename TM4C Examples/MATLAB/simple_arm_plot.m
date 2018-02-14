function simple_arm_plot(fig, jointangles)

%This is the function plotting the arms as a simulation by given the figure
%handle and a array of joint angles.

%***Choose the figure.
%figure(fig)
%figure
hold on
axis equal
axis([-2.5, 2.5, -2.5, 2.5])
axis off

%***Plot background.
plot(0, 0, 'Marker', '.', 'MarkerSize', 30, 'MarkerEdgeColor', 'k')
%For the convenience of refer 'line' function, the x and y values of points
%at the beginning and end of lines are devided into two matrices. Each
%column of each matrix represents the x or y values of two points in a
%single line.
pointsx = [0, -0.1, -0.1, -0.05,   0, -0.125, -0.075, -0.025, 0.025;
           0,  0.1, -0.05,    0, 0.05, -0.075, -0.025,  0.025, 0.075];
pointsy = [   0, -0.1, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15; 
           -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1];            
line(pointsx, pointsy, 'Color', 'k', 'LineWidth', 3);
line([0, 1], [0, 0], 'LineStyle', '--', 'Color', 'k', 'LineWidth', 3)

%%

%***Plot the moving parts.
dt = 0.04;
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
        P = [line([0, cos(jointangles(i, 1))], [0, sin(jointangles(i, 1))], 'Color', 'k', 'LineWidth', 3);
             line([cos(jointangles(i, 1)), cos(jointangles(i, 1)) + cos(jointangles(i, 1) + jointangles(i, 2))], [sin(jointangles(i, 1)), sin(jointangles(i, 1)) + sin(jointangles(i, 1) + jointangles(i, 2))], 'Color', 'k', 'LineWidth', 3);
             line([cos(jointangles(i, 1)), 2 * cos(jointangles(i, 1))], [sin(jointangles(i, 1)), 2 * sin(jointangles(i, 1))], 'LineStyle', '--', 'Color', 'k', 'LineWidth', 3);
             plot(cos(jointangles(i, 1)), sin(jointangles(i, 1)), 'Marker', '.', 'MarkerSize', 30, 'MarkerEdgeColor', 'k')]; 
         drawnow
    end
end