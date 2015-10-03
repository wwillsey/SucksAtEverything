function u = PID( handle, event, dt, goal_state, actual_state)
global robot x_cmd ei tp enc_init error error_prev stop;
%input parameter
kp = .5;
kd = 0.3;
ki = 0.01;
u = 0;
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
    
    if  abs(error) <= .0005
        u = 0;
    end
    
end
end

