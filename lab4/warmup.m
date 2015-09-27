global enc_init x_cmd ei tp overall_time my_plot my_plot2;

x_plot = 0;
y_plot = 0;
figure(2);
my_plot = plot(0, 0, 0 ,0, 'b-');
figure(3);
my_plot2 = plot(0, 0, 'b-');
 
enc_init = double(robot.encoders.LatestMessage.Left) / 1e3;
x_cmd = 0.1;
ei = 0;
overall_time = 0;
error = -1;

lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);