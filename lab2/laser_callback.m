function  laser_callback( src, msg)
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here
nearest_obj = lidar_reading(msg);
velocity_to_robot(nearest_obj);
pause(0.01);
% if(res(1) > 1)
%     vel = min((res(1)-1), 0.15);
% else
%     vel = max(res(1)-1, -0.15);
% end


% pause(0.001)
end

