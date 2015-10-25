global robot_trajectory1 robot_trajectory2 robot_trajectory3;
global running stop_timer;
global terminate;
global current;
global feed_back_plot1;
global system;
global data_log
global ran;
data_log = fopen('data_log', 'w');
ran = false;
figure(2)
f2 = plot(0,0,0,0);
xlabel('x');
ylabel('y');
feed_back_plot1 = plotter(f2);

current = 1;
terminate = false;

system = mrplSystem(robot, false);
stop_timer = -1;
running = true;

% lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);
enc = rossubscriber('/enc', @encoder_callback);
las = rossubscriber('/scan', @laser_callback);