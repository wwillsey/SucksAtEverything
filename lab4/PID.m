function u = PID( handle, event, dt, goal_state, actual_state)
global robot x_cmd ei tp enc_init error error_prev;
%input parameter
kp = 6;
kd = 1.5;
ki = 0.1;
u = 0;
current_enc = double(handle.LatestMessage.Left) / 1e3;
el_n = current_enc - enc_init;
actual_state = el_n;
t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
error_prev = error;
if(tp == -1)
    tp = t_now;
    error = goal_state - actual_state;
    error_prev = error;
else
    error = (goal_state - actual_state);
    max = .25;

        %Actual feed back loop
    ep = error *  kp;
    ed = ((error - error_prev) / dt) * kd; 
    ei = (ei + error * dt) * ki;
    u = ep + ed + ei;
    if u > max
        u = max;
    elseif u < -max
        u = -max;
    end
    
    if error <= .001
        u = 0;
    end
end
end

