function success = trajectoryExport(array)
name = [datestr(now,'mm_dd_yyyy_HH_MM_ss'), '.csv'];
csvwrite(name, array);
success = [];
end