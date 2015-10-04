function [ output_args ] = encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot;
global rtrajectory trajectory_follower fcontrol fbackcontrol;
global t_accum tp;
use_feedback = true;
el_n = double(handle.LatestMessage.Left)/1e3;
er_n = double(handle.LatestMessage.Right)/1e3;
if isempty(tp) || tp == -1
    tp = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
    fbackcontrol.init_state(el_n, er_n)
else
    t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
    dt = t_now - tp;
    if(dt < 0);
        dt = 1 + t_now - tp;
    end
    tp = t_now;
    t_accum = t_accum + dt;
    fbackcontrol.update_pose(el_n, er_n, dt);
    [vl, vr] = trajectory_follower.getVelocity(t_accum, use_feedback);
    robot.sendVelocity(vl, vr);
end

end

