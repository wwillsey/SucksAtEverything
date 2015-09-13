function [ vl, vr ] = velocity_to_robot( nearest_obj )
%VELOCITY_TO_ROBOT Summary of this function goes here
%   Detailed explanation goes here
x = nearest_obj(1);
% pub = rospublisher('/nxt_velocity', 'std_msgs/Float32MultiArray');
% y = nearest_obj(2);
% b = nearest_obj(3);
vl = 0;
vr = 0;

% x should never exceed 1.5 meters
if( x > 1.5 || x < 0.006)
    disp('Detected Distance is abnormal!!!!');
end
if( x > 1.05)
    vl = min(x - 1, 0.15); 
    vr = vl;
elseif (x < 0.95)
    vl = max(x - 1, -0.15);
    vr = vl;
else
    vl = 0;
    vr = 0;
end
% disp(2);
% dataMssg = rosmessage(pub);
% dataMssg.Data = [vl, vr];
% send(pub, dataMssg);
% pause(0.01);
end

