function  reference_trajectory_integrator( t_accum, amax, vmax, dist, sign, dt, t_del )
%REFERENCE_TRAJECTORY_INTEGRATOR Summary of this function goes here
%   Detailed explanation goes here
global accum_dist_ref; 
if isempty(accum_dist_ref)
    accum_dist_ref = 0;
else
    uref = trapezoidalVelocityProfile(t_accum - t_del, amax, vmax, dist, sign);
    accum_dist_ref = accum_dist_ref + uref*dt;
    reference_trajectory_plot(accum_dist_ref, t_accum);
end
end

