function [ output_args ] = encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot dist_accum;
persistent t_accum tp;
vmax = 0.25;
amax = 3*0.25;
dist = 1;
sgn = 1;
if isempty(t_accum)
    t_accum = 0;
end
if isempty(dist_accum)
    dist_accum = 0;
end
if isempty(tp)
    tp = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
else
    t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
    dt = t_now - tp;
    if(dt < 0)
        dt = 1+t_now - tp;
    end
    tp = t_now;
    t_accum = t_accum + dt;

    %PID(handle, event);
    u_ref = trapezoidalVelocityProfile(t_accum, amax, vmax, dist, sgn);
    reference_trajectory_integrator(t_accum, amax, vmax, dist, sgn, dt);
    actual_trajectory_integrator(handle, t_accum);
    robot.sendVelocity(u_ref, u_ref);
    pause(0.001);
end
end

