function PID( handle, event)
persistent error_prev error u ;
global robot x_cmd ei tp enc_init;
%input parameter
kp = 6;
kd = 0.1;
ki = 0.1;
current_enc = double(handle.LatestMessage.Left) / 1e3;
el_n = current_enc - enc_init;
t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
error_prev = error;
if(tp == -1) %initialization for the first time
    tp = t_now;
    error = (x_cmd - el_n);
    error_prev = error;
else if (abs(error_prev) > .001)
    dt = t_now - tp;
    tp = t_now;
    if (dt~=0) 
        if(dt < 0)
            dt = 1+t_now - tp;
        end
        error = (x_cmd - el_n);
        max = .25;

        %Actual feed back loop
%         ep = error *  kp;
%         ed = ((error - error_prev) / dt) * kd; 
%         ei = (ei + error * dt) * ki;
%         u = ep + ed + ei;
%         if u > max
%             u = max;
%         elseif u < -max
%             u = -max;
%         end
% 
%         if abs(error) <= .001
%             u = 0;
%         end
    end
    graph_error(error, dt);
    %robot.sendVelocity(u,u);
end

end

