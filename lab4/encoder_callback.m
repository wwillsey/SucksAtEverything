function [ output_args ] = encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot dist_accum tp t_accum;
global stop ;
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
if isempty(tp) || tp == -1 || stop == 2
    tp = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
else
    t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
    dt = t_now - tp;
    if(dt < 0)
        dt = 1+t_now - tp;
    end
    tp = t_now;
    t_accum = t_accum + dt;

    
    u_ref = trapezoidalVelocityProfile(t_accum, amax, vmax, dist, sgn);
   % u_del = trapezoidalVelocityProfile(t_accum - t_del, amax, vmax, dist, sgn)
    s = reference_trajectory_integrator(t_accum, amax, vmax, dist, sgn, dt, t_del);
    if(s >= dist - 0.001 ) && stop == 0
        tic;
        stop = 1;
    end
    if(stop == 1)
        stop_now = toc;
        if (stop_now) >= 1
            stop = 2;
        end
    end
    el_n = actual_trajectory_integrator(handle, t_accum);
    u_pid = PID(handle, event, dt, s, el_n);
    plot_final_trajectory(el_n, s, t_accum);
    u = u_ref + u_pid;
    if(abs(u) > 0.3)
        u = sign(u) * 0.25;
    end 
    if(stop == 2)
        u = 0;
        el_n
    end
    robot.sendVelocity(u, u);
    %pause(0.001);
end
end

