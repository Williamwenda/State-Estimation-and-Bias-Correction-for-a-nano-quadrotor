function [aligned_msgs]=align(msgs1,time1,msgs2,time2)
%% Align data based on their timestep
%  msgs2 are aligned with msgs1
i1=1;
i2=0;
aligned_msgs=[];
for i1=1:length(time1)
    while i2+1 < length(time2) && time2(i2+1)<time1(i1)
        i2=i2+1;
        aligned_msgs=[aligned_msgs;msgs1(i1,:)];
    end
end
%% After alignment, use time1 as the timestep.

end