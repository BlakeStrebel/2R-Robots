function data = PIDGet(state, motor)
if motor == 1
    data(1) = state.Kp(1);
    data(2) = state.Ki(1);
    data(3) = state.Kd(1);
elseif motor == 2
    data(1) = state.Kp(2);
    data(2) = state.Ki(2);
    data(3) = state.Kd(2); 
else
    data(1) = state.Kp(1);
    data(2) = state.Ki(1);
    data(3) = state.Kd(1);  
    data(4) = state.Kp(2);
    data(5) = state.Ki(2);
    data(6) = state.Kd(2);    
end
end