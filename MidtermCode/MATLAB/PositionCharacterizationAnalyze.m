% Forward direction data
desired_pos = desPos1(1915:end)/16383*360;
actual_pos = absPos1(1915:end)/16383*360;
d = pwm1(1915:end)/9600*100;
dmax = -101*ones(1,size(desired_pos,2));
dmin = 101*ones(1,size(desired_pos,2));


figure; plot(desired_pos, d); title('Duty Cycle vs. Desired Position')




% find duplicates of actual_pos (stagnation points)
% Store the maximum and minimum duty cycles at those points
% Averages are cogging waveforms
% Half max difference is duty cycle req to overcome max dt + st
%   Duty cycles below this value -> just overcoming st + ds
%   Averatge these values to get ddtst
% All duty cycles above ddtst are overcoming stiction only
%   -> Find the mean, subtract from ddtst to get ddt


count = zeros(size(d));
for i = 1:size(desired_pos,2)
    for j = 1:size(actual_pos,2)
        if (desired_pos(i) == actual_pos(j))
            count(i) = count(i) + 1;
            
            if (d(j) > dmax(i))
                dmax(i) = d(j);
            end
            if (d(j) < dmin(i))
                dmin(i) = d(j);
            end
            
        end
    end
    ddtsti(i) = (dmax(i) - dmin(i))/2;
    dcogi(i) = (dmax(i) + dmin(i))/2;
end


index = find(count > 1);
figure; plot(desired_pos(index),dcogi(index));
title('Cogging Torque vs. Desired Position')

ddtstmax = max(ddtsti);

k = 0; ddtstktemp = 0;
for i = index
    if (ddtstmax > dmax(i) || -ddtstmax < dmin(i))
        ddtstktemp = ddtstktemp + ddtsti(i);
        k = k + 1;
    end
end

ddtst = ddtstktemp/k;

k = 0; dstktemp = 0;
for i = index
   if (ddtstmax < dmin(i) || -ddtstmax > dmax(i))
       dstktemp = dstktemp + ddtsti(i);
       k = k + 1;
   end
end

dst = dstktemp/k;
ddt = ddtst - dst;



%  
% k = 1;
% for i = 1:size(desired_pos,2)
%     
%     for j = 1:size(actual_pos,2)
%         
%         if (desired_pos(i) == actual_pos(j))
%             
%             if (d(j) > dmax(i))
%                 dmax(i) = d(j);
%             end
%             if (d(j) < dmin(i))
%                 dmin(i) = d(j);
%             end
%             
%         end
%     end
%     if (dmax(i) ~= -101 && dmin(i) ~= 101)
%         index(k) = i;
%         ddtst(k) = 0.5*(dmax(i) - dmin(i));
%         dcog(k) = 0.5*(dmax(i) - dmin(i));
%         k = k + 1;
%     end
% end
% 
% figure; hold on;
% plot(desired_pos(index), dmax(index),'b');
% plot(desired_pos(index), dmin(index),'r');
% 
% 
% figure; plot(desired_pos(index), dcog);
% 
% ddtstmax = max(ddtst);
%  
% k = 0;
% dstktemp = 0;
% for i = 1:size(dcog,2)
%     if ddtst(i) < ddtstmax || ddtst(i) > -ddtstmax
%         dstktemp = dstktemp + ddtst(i);
%         k = k + 1;
%      end
%  end
%  ddtst_bar = dstktemp/k;
%  
%  k = 0;
%  dsttemp = 0;
%  for i = 1:size(dcog,2)
%      if ddtstmax < dmin(i) || -ddtstmax > dmax(i)
%          dsttemp = dsttemp + ddtst(i);
%          k = k + 1;
%      end
%  end
%  
%  dst_bar = dsttemp/k;
%  ddt = ddtst_bar - dst_bar;





  