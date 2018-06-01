function statenew = positionSave(stateold, data, motor)
statenew = stateold;
if motor == 1
    statenew.p(1) = data(1);
elseif motor == 2
    statenew.p(2) = data(1);
else
    statenew.p = [data(1), data(3)];
end

end