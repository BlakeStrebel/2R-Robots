function statenew = defaultPID(stateold)
    statenew = stateold;
    statenew.Kp = [1, 2];
    statenew.Ki = [3, 4];
    statenew.Kd = [5, 6];
end