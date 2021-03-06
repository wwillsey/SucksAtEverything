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
global laser_counter use_localization;
global tp;
global waiting acquiring;

waiting = false;
acquiring = false;
close all;

data_log = fopen('data_log', 'w');
ran = false;
%% Plotting data and global variable init
figure(2)
f2 = plot(0,0,0,0);
xlabel('x');
ylabel('y');
feed_back_plot1 = plotter(f2);


laser_counter = 0;
current = 1;
terminate = true;
tp = -1;
use_localization = true;
stop_timer = -1;
running = true;

%% The walls
p1 = [0;4];
p2 = [0;0];
p3 = [4;0];
lines_p1 = [p1 p2];
lines_p2 = [p2 p3];

%% Init system and guess a pose
robotPose = [0.5, 0.5, pi/2];
system = mrplSystem(robot, lines_p1, lines_p2, robotPose, true);

%% refine pose before robot starts
lidar_read = robot.laser.LatestMessage.Ranges;
rImage = rangeImage(lidar_read, 10, true);
x = rImage.xArray;
y = rImage.yArray;
pts = [x;y;ones(1, rImage.numPix)];
[succ, p_lid] = system.map.refinePose(pose(robotPose), pts, 200);
robot_init_pose = p_lid.getPoseVec';
system.estRobot.init_pose = robot_init_pose;
system.estRobot.robot_pose_fus = robot_init_pose;

cp = robot_init_pose;
h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
fp = h^-1 * [0.25;0.75;1];
%robot_trajectory1 = cubicSpiral.planTrajectory(fp(1), fp(2), pi/2-cp(3), 1);
%robot_trajectory1.planVelocities(0.15);
%system.trajectoryFollower.loadTrajectory(robot_trajectory1, robot_init_pose);

%% Start sensor callbacks
enc = rossubscriber('/enc', @encoder_callback);
las = rossubscriber('/scan', @laser_callback);