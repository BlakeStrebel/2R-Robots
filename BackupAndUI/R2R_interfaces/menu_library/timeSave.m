function statenew = timeSave(stateold, data)
statenew = stateold;
statenew.t = floor(data(1) * 1000);
end