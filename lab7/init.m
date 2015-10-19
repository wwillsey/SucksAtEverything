global robot_trajectory1 robot_trajectory2 robot_trajectory3;
global running stop_timer;
global terminate;
global current;
global feed_back_plot1;
global system;
global data_log

data_log = fopen('data_log', 'w');

figure(2)
f2 = plot(0,0,0,0);
xlabel('x');
ylabel('y');
feed_back_plot1 = plotter(f2);

current = 1;
terminate = false;

robot_trajectory1 = cubicSpiral.planTrajectory(0.25, 0.25, 0, 1);
robot_trajectory1.planVelocities(0.2);
robot_trajectory2 = cubicSpiral.planTrajectory(-0.5, -0.5, -pi/2, 1);
robot_trajectory2.planVelocities(0.2);
robot_trajectory3 = cubicSpiral.planTrajectory(-0.25, 0.25, pi/2, 1);
robot_trajectory3.planVelocities(0.2);

system = mrplSystem(robot, true);
system.trajectoryFollower.loadTrajectory(robot_trajectory1, [0,0,0]);
stop_timer = -1;
running = true;

lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);
%enc = rossubscriber('/enc', @encoder_callback);