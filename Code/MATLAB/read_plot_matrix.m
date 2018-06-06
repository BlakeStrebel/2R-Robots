function data = read_plot_matrix(mySerial,ref1,ref2)
% Reads in data from PIC32 during position trajectory execution
% data(:,1) = reference position motor 1
% data(:,2) = reference position motor 2
% data(:,3) = actual position motor 1
% data(:,4) = actual position motor 2
% data(:,5) = control effort motor 1
% data(:,6) = control effort motor 2


nsamples = fscanf(mySerial,'%d');       % first get the number of samples being sent
data = zeros(nsamples,6);               
figure 
hold on
title('Live plot')
h = animatedline('Color','r');
r = animatedline('Color','y');
axis([0,nsamples*10,-200,200])
%legend('reference angle','actual angle')
x  = 10*(1:size(data,1));
data(:,1) = ref1(1:size(data,1));
data(:,2) = ref2(1:size(data,1));


for i=1:nsamples
    data(i,3:6) = fscanf(mySerial,'%d %d %d %d'); % read in data from Tiva (pos m1, pos m2, control eff 1, control eff 2)
    
    xvec=x(i);
    yrvec = data(i,4)/16383*360;
    yvec = data(i,3)/16383*360;
    addpoints(r,xvec,yrvec)
    addpoints(h,xvec,yvec)
    drawnow limitrate
    
end
hold off
save('temp.mat','data');

%data(:,1) = ref1(1:size(data,1));
%data(:,2) = ref2(1:size(data,1));

% Perform Conversions
data(:,1:4) = data(:,1:4)/16383*360;    % convert counts to degrees
data(:,5:6) = data(:,5:6)/2047*100;     % Scale control effort from 0 -> 100
times = 10*(1:size(data,1)); % adjust this if data is decimated

if nsamples > 1						        
    figure;
    stairs(times,data(:,[1,3,5]));            
    score = mean(abs(data(:,1)-data(:,3)));
    title(sprintf('Motor 1, Avg error: %5.1f mm',score))
    ylabel('Angle (degrees)')
    xlabel('Time (ms)')
    legend('reference angle','actual angle', 'control effort')
    
    figure;
    stairs(times,data(:,[2,4,6]));            
    score = mean(abs(data(:,2)-data(:,4)));
    title(sprintf('Motor 2, Avg error: %5.1f mm',score))
    ylabel('Angle (degrees)')
    xlabel('Time (ms)')
    legend('reference angle','actual angle', 'control effort')
else
    fprintf('Only 1 sample received\n')
    disp(data);
end

end