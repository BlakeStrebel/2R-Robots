s1 = serial('COM10', 'BaudRate', 128000, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none');
fopen(s1);
r = 10
while r>0
    sprintf(fscanf(s1))
    r = r -1;
end
fclose(s1);