function statenew = PIDSave(stateold, data, motor)
statenew = stateold;
if motor == 1
    statenew.Kp(1) = data(1);
    statenew.Ki(1) = data(2);
    statenew.Kd(1) = data(3);
elseif motor == 2
    statenew.Kp(2) = data(1);
    statenew.Ki(2) = data(2);
    statenew.Kd(2) = data(3);   
else
    statenew.Kp(1) = data(1);
    statenew.Ki(1) = data(2);
    statenew.Kd(1) = data(3);
    statenew.Kp(2) = data(4);
    statenew.Ki(2) = data(5);
    statenew.Kd(2) = data(6);    
end
end