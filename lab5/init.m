close all;
global  rtrajectory trajectory_follower fcontrol fbackcontrol;
global t_accum tp;
global plot1 plot2;
tp = -1;
t_accum = 0;

fcontrol = figure8ReferenceControl(0.3, 0.3, 2);
rtrajectory = robotTrajectory(fcontrol, 0, 18, [0,0,0]);
fbackcontrol = controller();
trajectory_follower = trajectoryFollower(fcontrol, rtrajectory, fbackcontrol);
plot1 = plotter(2);
plot2 = plotter(3);
%lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);
enc = rossubscriber('/enc', @encoder_callback);