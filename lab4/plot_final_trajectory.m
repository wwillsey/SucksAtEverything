function plot_final_trajectory( el_n, s, t_accum )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global t_array s_array e_array d_array;
    t_array = [t_array, t_accum];
    s_array = [s_array, s];
    e_array = [e_array, el_n];
    d_array = [d_array, el_n - s];
    if(t_accum < 10)
        figure(2);
        plot(t_array, s_array, t_array, e_array, 'b-');
         figure(3); 
         plot(t_array, d_array, 'b-');
    end
end

