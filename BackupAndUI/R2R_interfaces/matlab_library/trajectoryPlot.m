function data = trajectoryPlot(t, motor)
if motor == 1 
    figure
	data = ones(t, 1) * 1;
    plot(data);
elseif motor == 2
    data = ones(t, 1) * 2; 
    plot(data);  
else
    data = [ones(t, 1) * 3, ones(state.t, 1) * 4];
    subplot(1, 2, 1)
    plot(data(:, 1))
    subplot(1, 2, 2)
    plot(data(:, 2))    
end
end