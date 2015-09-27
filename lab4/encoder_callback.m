function [ output_args ] = encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot dist_accum tp t_accum;
vmax = 0.25;
amax = 2.5*0.25;
dist = 1;
sgn = 1;
t_del = 0.5;
if isempty(t_accum)
    t_accum = 0;
end
if isempty(dist_accum)
    dist_accum = 0;
end
if isempty(tp) || tp == -1
    tp = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
else
    t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
    dt = t_now - tp;
    if(dt < 0)
        dt = 1+t_now - tp;
    end
    tp = t_now;
    t_accum = t_accum + dt;

    u_ref = PID(handle, event, dt, 1, 00);
    %u_ref = trapezoidalVelocityProfile(t_accum, amax, vmax, dist, sgn);
    %reference_trajectory_integrator(t_accum, amax, vmax, dist, sgn, dt, t_del);
    el_n = actual_trajectory_integrator(handle, t_accum);
    %u_del = trapezoidalVelocityProfile(t_accum - t_del, amax, vmax, dist, sgn);
    %plot_difference(el_n, u_del, t_accum);
    robot.sendVelocity(u_ref, u_ref);
    %pause(0.001);
end
end

