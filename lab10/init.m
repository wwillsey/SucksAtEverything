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
figure(2)
f2 = plot(0,0,0,0);
xlabel('x');
ylabel('y');
feed_back_plot1 = plotter(f2);

current = 1;
terminate = false;


stop_timer = -1;
running = true;

robot_trajectory1 = cubicSpiral.planTrajectory(0.25, 0.75, pi/2, 1);
robot_trajectory1.planVelocities(0.2);
p1 = [0;4];
p2 = [0;0];
p3 = [4;0];
lines_p1 = [p1 p2];
lines_p2 = [p2 p3];
robotPose = [0.5, 0.5, pi/2];
system = mrplSystem(robot, lines_p1, lines_p2, robotPose, true);
system.trajectoryFollower.loadTrajectory(robot_trajectory1, robotPose);
enc = rossubscriber('/enc', @encoder_callback);
las = rossubscriber('/scan', @laser_callback);