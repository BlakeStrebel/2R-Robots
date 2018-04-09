function [steplog, data] = inputProcess(oldstep, options, symbol, olddata)

data = olddata;
if options{1} == -1 && length(symbol) ~= 1
	data = str2num(symbol);
    steplog = [oldstep, 'X'];
elseif (strcmp(oldstep, 'at1dXyy') || strcmp(oldstep, 'at1dXyn')) && symbol == 'y'
    steplog = 'at1d';
elseif (strcmp(oldstep, 'at1dXyy') || strcmp(oldstep, 'at1dXyn')) && symbol == 'n'
    steplog = 'a';
else
    switch symbol
        case {'q', 'a'}
            steplog = symbol;
        case 'r'
            steplog = oldstep(1: end - 1);
        case options
            steplog = [oldstep, symbol];
        otherwise
            steplog = oldstep;
    end
end
end