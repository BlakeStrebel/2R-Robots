function data = trajectoryTracking(state, motor, serial)
figure;
hold on;
time = 50: 50: state.t;
if motor == 3  
    subplot(2, 1, 1);
    plot(time / 1000, state.dtraj(time, 1)', 'r');
    fig1 = animatedline('Color', 'b');
    title('Theta 1 plots');
    xlabel('Time (s)');
    xlim([0, state.t / 1000]);
    ylabel('Angle (rads)');
    ylim([-2 * pi, 2 * pi]); 
    yyaxis right;
    ylabel('Angle (degrees)');
    ylim([-360, 360]);
    subplot(2, 1, 2);
    plot(time / 1000, state.dtraj(time, 2)', 'r');
    fig2 = animatedline('Color', 'b');
    title('Theta 2 plots');
    xlabel('Time (s)');
    xlim([0, state.t / 1000]);
    ylabel('Angle (rads)');
    ylim([-2 * pi, 2 * pi]); 
    yyaxis right;
    ylabel('Angle (degrees)');
    ylim([-360, 360]);       
    data = zeros(state.t, 2);
    value = [0, 0];
    fwrite(serial, 22);
    for i = 1: state.t / 10
        value(1) = readFloat(serial);
        value(2) = readFloat(serial);
        data(i, :) = value;
        if rem(i, 5) == 0
            addpoints(fig1, i / 100, value(1));
            addpoints(fig2, i / 100, value(2));
            drawnow limitrate;
        end
    end 
    subplot(2, 1, 1);
    legend('Desired', 'Actual');
    subplot(2, 1, 2);
    legend('Desired', 'Actual');
else
    plot(time / 1000, state.dtraj(time, motor)', 'r');
    fig = animatedline('Color', 'b');
    xlabel('Time (s)');
    xlim([0, state.t / 1000]);
    ylabel('Angle (rads)');
    ylim([-2 * pi, 2 * pi]); 
    yyaxis right;
    ylabel('Angle (degrees)');
    ylim([-360, 360]);   
    legend('Desired', 'Actual');
    data = zeros(state.t, 1);
    if motor == 1
        fwrite(serial, 20);
        title('Motor 1 position');
    else
        fwrite(serial, 21);
        title('Motor 2 position');
    end
    for i = 1: state.t / 10
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