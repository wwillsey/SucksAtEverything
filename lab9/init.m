global robot_trajectory1 robot_trajectory2 robot_trajectory3;
global running stop_timer;
global terminate;
global current;
global feed_back_plot1;
global system;
global data_log
global ran;
global map;
global robotPose;
close all;
data_log = fopen('data_log', 'w');
ran = false;
% figure(2)
% f2 = plot(0,0,0,0);
% xlabel('x');
% ylabel('y');
% feed_back_plot1 = plotter(f2);

current = 1;
terminate = false;

system = mrplSystem(robot, true);
stop_timer = -1;
running = true;

p1 = [0;4];
p2 = [0;0];
p3 = [4;0];
lines_p1 = [p1 p2];
lines_p2 = [p2 p3];
robotPose = pose(15*0.0254, 9*0.0254, 0/2.0);
map = lineMapLocalizer(lines_p1, lines_p2, .01, .001, .0005);
% lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);
%enc = rossubscriber('/enc', @encoder_callback);
las = rossubscriber('/scan', @laser_callback);