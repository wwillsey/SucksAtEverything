function [ output_args ] = plot_difference( el_n, u_del, t_accum )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global difference_plot accum_dist_ref;
if(t_accum <= 7)
    set(difference_plot, 'xdata', [get(difference_plot, 'xdata') t_accum], 'ydata', [get(difference_plot, 'ydata') (el_n-accum_dist_ref)]);
end
end

