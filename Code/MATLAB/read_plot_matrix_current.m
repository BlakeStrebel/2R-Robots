function data = read_plot_matrix_current(mySerial)
% Reads in data from PIC32 during position trajectory execution
% data(:,1) = 
% data(:,2) = reference position motor 2
% data(:,3) = actual position motor 1
% data(:,4) = actual position motor 2
% data(:,5) = control effort motor 1
% data(:,6) = control effort motor 2


nsamples = fscanf(mySerial,'%d');       % first get the number of samples being sent
data = zeros(nsamples,4);               
 
for i=1:nsamples
    data(i,1:4) = fscanf(mySerial,'%d %d %f %f'); % read in data from Tiva (pos m1, pos m2, control eff 1, control eff 2)   
end
hold off
save('temp.mat','data');

% Perform Conversions
%data(:,1:4) = data(:,1:4)/16383*360;    % convert counts to degrees
%data(:,5:6) = data(:,5:6)/9600*100;     % Scale control effort from 0 -> 100
%times = 10*(1:size(data,1)); % adjust this if data is decimated

if nsamples > 1						        
    figure;
    plot(data(:,1:4));
else
    fprintf('Only 1 sample received\n')
    disp(data);
end

end