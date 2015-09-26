function [ u ] = PID( handle, event)
persistent error_prev error;
global robot x_cmd ei tp;
kp = 3;
kd = 2;
ki = 0.1;
x_cur = double(handle.LatestMessage.Left) / 1e3;
error_prev = error;
t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
dt = t_now - tp;
tp = dt;
if el_n ~= 0 && er_n ~= 0 &&  dt~=0
if(dt < 0)
    dt = 1+t_now - rs_p.time;
end
error = (x_cmd - x_cur);
max = .25;

ed = (error - error_prev) / dt; 
ei = ei + error * dt;
u = error*kp + ed*kd + ei*ki;
if u > max
    u = max;
elseif u < -max
    u = -max;
end

if error <= .001
    u = 0;
end
robot.sendVelocity(u,u);
end

