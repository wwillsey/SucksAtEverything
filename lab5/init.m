close all;
global  rtrajectory trajectory_follower fcontrol fbackcontrol;
global t_accum tp;
global plot3 plot2 plot4;
tp = -1;
t_accum = 0;

fcontrol = figure8ReferenceControl(0.3, 0.3, 2);
rtrajectory = robotTrajectory(fcontrol, 0, 18, [0,0,0]);
fbackcontrol = controller();
trajectory_follower = trajectoryFollower(fcontrol, rtrajectory, fbackcontrol);
figure(2)
f2 = plot(0,0,0,0);
figure(3)
f3 = plot(0,0,0,0);
figure(4)
f4 = plot(0,0,0,0,0,0);
plot2 = plotter(f2);
plot3 = plotter(f3);
plot4 = plotter(f4);
%lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);
enc = rossubscriber('/enc', @encoder_callback);