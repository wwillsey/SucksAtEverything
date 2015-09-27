function [ output_args ] = graph_error(error, time_step)
%GRAPH_ERROR Summary of this function goes here
%   Detailed explanation goes here
global overall_time my_plot;
overall_time = overall_time + time_step;
if(abs(error) > 0.001)
    set(my_plot, 'xdata', [get(my_plot, 'xdata') overall_time], 'ydata', [get(my_plot, 'ydata') error]);
end
end

