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

for i=1:nsamples
    data(i,3:6) = fscanf(mySerial,'%d %d %f %f'); % read in data from Tiva
end

save('temp.mat','data');

data(:,1) = ref1(1:size(data,1));
data(:,2) = ref2(1:size(data,1));

% Perform Conversions
data(:,1:4) = data(:,1:4)/16383*360;    % convert counts to degrees
data(:,5:6) = data(:,5:6)/9600*100;     % Scale control effort from 0 -> 100
times = 10*(1:size(data,1)); % adjust this if data is decimated

if nsamples > 1						        
    figure;
    stairs(times,data(:,[1,3,5]));            
    score = mean(abs(data(:,1)-data(:,3)));
    title(sprintf('Motor 1, Avg error: %5.1f mm',score))
    ylabel('Position (mm)')
    xlabel('Time (ms)')
    legend('reference angle','actual angle', 'control effort')
    
    figure;
    stairs(times,data(:,[2,4,6]));            
    score = mean(abs(data(:,2)-data(:,4)));
    title(sprintf('Motor 1, Avg error: %5.1f mm',score))
    ylabel('Position (mm)')
    xlabel('Time (ms)')
    legend('reference angle','actual angle', 'control effort')
else
    fprintf('Only 1 sample received\n')
    disp(data);
end

end