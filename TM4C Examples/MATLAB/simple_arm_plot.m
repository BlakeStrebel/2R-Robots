
%***Set figure size.
figure
hold on
set(gcf,'Units', 'pixels', 'PaperSize', [1920, 1080], 'PaperPositionMode','auto', 'Position', [0, 0, 1920, 1080], 'Color', 'k');
axis equal
%Axis ranges are hard to estimate and they are based on the joint angles. 
%They should be be modified manually. 
axis([-3.0654, 7.0778, -1.5, 4.5])
axis off

%***Plot background.
plot(0, 0, 'Marker', '.', 'MarkerSize', 30, 'MarkerEdgeColor', 'w')
%For the convenience of refer 'line' function, the x and y values of points
%at the beginning and end of lines are devided into two matrices. Each
%column of each matrix represents the x or y values of two points in a
%single line.
pointsx = [0, -0.1, -0.1, -0.05,   0, -0.125, -0.075, -0.025, 0.025;
           0,  0.1, -0.05,    0, 0.05, -0.075, -0.025,  0.025, 0.075];
pointsy = [   0, -0.1, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15; 
           -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1];            
line(pointsx, pointsy, 'Color', 'w', 'LineWidth', 3);
line([0, 1], [0, 0], 'LineStyle', '--', 'Color', 'w', 'LineWidth', 3)

%%
P = [];
theta = [pi / 6, pi / 3];
delete(P);
P = [line([0, cos(theta(1))], [0, sin(theta(1))], 'Color', 'w', 'LineWidth', 3);
     line([cos(theta(1)), cos(theta(1)) + cos(theta(1) + theta(2))], [sin(theta(1)), sin(theta(1)) + sin(theta(1) + theta(2))], 'Color', 'w', 'LineWidth', 3);
     line([cos(theta(1)), 2 * cos(theta(1))], [sin(theta(1)), 2 * sin(theta(1))], 'LineStyle', '--', 'Color', 'w', 'LineWidth', 3);
     plot(cos(theta(1)), sin(theta(1)), 'Marker', '.', 'MarkerSize', 30, 'MarkerEdgeColor', 'w')];   

