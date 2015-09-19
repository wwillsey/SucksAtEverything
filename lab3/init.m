robot = neato('sim');
global rs_p rs_n my_plot
rs_p = struct('time', 0, 'el', 0, 'er',0, 'theta', 0, 'x', 0, 'y', 0);
rs_n = struct('time', 0, 'el', 0, 'er',0, 'theta', 0, 'x', 0, 'y', 0);
x_plot = 0;
y_plot = 0;
my_plot = plot(x_plot, y_plot, 'b-');
lh = event.listener(robot.encoders, 'OnMessageReceived', @encoderCallback);
