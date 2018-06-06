function statenew = velocitySave(stateold, data, motor)
statenew = stateold;
if motor == 1
    statenew.v(1) = data(1);
elseif motor == 2
    statenew.v(2) = data(1);
else
    statenew.v = [data(1), data(2)];
end

end