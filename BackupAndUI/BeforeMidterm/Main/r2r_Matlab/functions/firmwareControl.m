function matrix = firmwareControl(s, steplog, data)

matrix = [];
%fwrite(s, steplog(end))

switch steplog        
    case 'at1dX'
    	writeArray(s, data);
        while s.BytesAvailable == 0
        end
        disp('11111111111111')
        if fread(s,1) == 'p'
            disp('11111111111111')
            writeArray(s, trajGenerator());
            while s.BytesAvailable == 0
            end
        end
        
    case 'at1dXy'            
            if fread(s,1) == 'p'
                fwrite(s,'s');
                while s.BytesAvailable ~= 40000
                end
                matrix = readArray(s, 10000, 1);
                figure
                plot(matrix)
                csvwrite('test.csv',matrix)
            end
end

end               
