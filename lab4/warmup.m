global enc_init x_cmd ei tp overall_time my_plot my_plot2 dist_accum t_accum;
global accum_dist_ref final_plot stop s_array t_array e_array d_array use_pid;
global stop_start;
use_pid = 1;
accum_dist_ref = 0;
s_array = 0;
t_array = 0;
e_array = 0;
d_array = 0;
stop = 0;
x_plot = 0;
y_plot = 0;
figure(2);
dist_accum = 0;
t_accum = 0;
enc_init = double(robot.encoders.LatestMessage.Left) / 1e3;
x_cmd = 0.1;
ei = 0;
overall_time = 0;
tp = -1;
error = -1;

%lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);
enc = rossubscriber('/enc',@encoder_callback);
