function statenew = trajectoryDefault(stateold)
statenew = stateold;
statenew.t = 10;
statenew.dtraj = ones(statenew.t * 1000, 2) * pi / 3;
end