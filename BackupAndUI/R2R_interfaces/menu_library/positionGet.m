function data = positionGet(state, motor)
if motor == 1
    data = state.p(1);
elseif motor == 2
    data = state.p(2);
else
    data = state.p;
end
end