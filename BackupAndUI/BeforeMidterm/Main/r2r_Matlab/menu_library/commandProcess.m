function [cmdlognew, data] = commandProcess(cmdlogold, response, options)
data = [];
cmdlognew = cmdlogold;
if length(response) > 1
    if options(1) == 'X'
        data = str2num(response);
        if ~isempty(data)
            cmdlognew = [cmdlogold, 'X'];
        end 
    end    
elseif isempty(response)
    cmdlognew = cmdlogold;
elseif response == 'a' || 'q'
	cmdlognew = response;
elseif response == 'r' && options(end) == 'r'
    cmdlognew = cmdlogold(1: end - 1);
elseif ismember(response, options)
    cmdlognew = [cmdlogold, response];    
end