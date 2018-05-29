function data = positionGet(state, motor)
if motor == 1
    data = state.v(1);
elseif motor == 2
    data = state.v(2);
else
    data = state.v;
end
end