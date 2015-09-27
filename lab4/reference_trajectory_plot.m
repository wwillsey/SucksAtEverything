function [ output_args ] = reference_trajectory_plot( accum_dist, t_accum )
%REFERENCE_TRAJECTORY_PLOT Summary of this function goes here
%   Detailed explanation goes here
global my_plot;
if(t_accum <= 7)
    set(my_plot, 'xdata', [get(my_plot, 'xdata') t_accum], 'ydata', [get(my_plot, 'ydata') accum_dist]);
end
end

