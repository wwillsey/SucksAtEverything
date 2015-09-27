function [ output_args ] = reference_trajectory_integrator( t_accum, amax, vmax, dist, sign, dt )
%REFERENCE_TRAJECTORY_INTEGRATOR Summary of this function goes here
%   Detailed explanation goes here
persistent accum_dist_ref; 
if isempty(accum_dist_ref)
    accum_dist_ref = 0;
else
    uref = trapezoidalVelocityProfile(t_accum, amax, vmax, dist, sign);
    accum_dist_ref = accum_dist_ref + uref*dt;
    reference_trajectory_plot(accum_dist_ref, t_accum);
end
end

