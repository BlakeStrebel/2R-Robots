function data = trajectoryPlot(t, motor, serial)
figure;
hold on;
if motor == 3  
    fig1 = animatedline('Color', 'r');
    fig2 = animatedline('Color', 'b');
    title('Position plots');  
    xlabel('Time (s)');
    xlim([0, t / 1000]);
    ylabel('Angle (rads)');
    ylim([-2 * pi, 2 * pi]); 
    yyaxis right;
    ylabel('Angle (degrees)');
    ylim([-360, 360]);     
    data = zeros(t, 2);
    value = [0, 0];
    fwrite(serial, 12);
    for i = 1: t / 10
        value(1) = readFloat(serial);
        value(2) = readFloat(serial);
        data(i, :) = value;
        if rem(i, 5) == 0
            addpoints(fig1, i / 100, value(1));
            addpoints(fig2, i / 100, value(2));
            drawnow limitrate;
        end
    end 
    legend('Motor 1', 'Motor 2');
else
    fig = animatedline; 
    title('Position plot');  
    xlabel('Time (s)');
    xlim([0, t / 1000]);
    yyaxis right;
    ylabel('Angle (degrees)');
    ylim([-360, 360]);
    yyaxis left;
    ylabel('Angle (rads)');
    ylim([-2 * pi, 2 * pi]);      
    data = zeros(t, 1);
    if motor == 1
        fwrite(serial, 10);
        legend('Motor 1 position');
    else
        fwrite(serial, 11);
        legend('Motor 2 position');
    end
    for i = 1: t / 10
        value = readFloat(serial);
        data(i) = value;
        if rem(i, 5) == 0
            addpoints(fig, i / 100, value);
            drawnow limitrate;
        end
    end    
end
hold off;
end