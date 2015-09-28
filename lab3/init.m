global rs_p rs_n my_plot
el = double(robot.encoders.LatestMessage.Left)/1e3;
er = double(robot.encoders.LatestMessage.Right)/1e3;
time = double(robot.encoders.LatestMessage.Header.Stamp.Nsec) / 1e9;
rs_p = struct('time', time, 'el', el, 'er',er, 'theta', 0, 'x', 0, 'y', 0);
rs_n = struct('time', time, 'el', el, 'er',er, 'theta', 0, 'x', 0, 'y', 0);

x_plot = 0;
y_plot = 0;
my_plot = plot(x_plot, y_plot, 'b-');
%axis equal;
%robot.encoders.NewMessageFcn = @encoderCallback;
enc = rossubscriber('/enc', @encoderCallback);

%robot.encoders.NewMessageFcn=@encoderCallback;
%lh = event.listener(robot.encoders, 'OnMessageReceived', @encoderCallback);
drive_robot;