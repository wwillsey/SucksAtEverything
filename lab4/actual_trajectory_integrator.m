function [ output_args ] = actual_trajectory_integrator( handle, t_accum )
%ACTUAL_TRAJECTORY_INTEGRATOR Summary of this function goes here
%   Detailed explanation goes here
global enc_init my_plot2;

% NOT the real thing
current_enc = double(handle.LatestMessage.Left) / 1e3;
el_n = current_enc - enc_init;

if(t_accum <= 5)
    X = 
    set(my_plot2, 'xdata', [get(my_plot2, 'xdata') t_accum], 'ydata', [get(my_plot2, 'ydata') el_n]);
end
end

