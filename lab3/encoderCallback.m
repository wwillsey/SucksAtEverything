function [ output_args ] = encoderCallback( handle, event )
%ENCODERCALLBACK Summary of this function goes here
%   Detailed explanation goes here
global rs_n rs_p my_plot;

t_now = event.LatestMessage.Header.Stamp.Nsec / 1e9;
el_n = event.LatestMessage.Left/1e3;
er_n = event.LatestMessage.Right/1e3;
dt = t_now - rs_p.time;
del = el_n - rs_p.el;
der = er_n - rs_p.er;
vl_n = del / dt;
vr_n = der / dt;
w = (vr_n-vl_n)/0.235;
V = (vr_n+vl_n)/2;
theta_p = rs_p.theta;
theta_n = theta_p + w * dt;
dx = cos(theta_n) * V * dt;
dy = sin(theta_n) * V * dt;
x_p = rs_p.x;
y_p = rs_p.y;
x_n = x_p + dx;
y_n = y_p + dy;
rs_p = rs_n;
rs_n = struct('time', t_now, 'el', el_n, 'er', er_n, 'theta', theta_n, 'x', x_n, 'y', y_n);

set(my_plot, 'xdata', [get(my_plot, 'xdata') x_n], 'ydata', [get(my_plot, 'ydata') y_n]);
pause(0.0001);
end

