global  rtrajectory trajectory_follower fcontrol fbackcontrol;
global t_accum tp;
tp = -1;
t_accum = 0;

fcontrol = figure8ReferenceControl(0.4, 0.4, 1);
rtrajectory = robotTrajectory(fcontrol, 0, 15, [0,0,0]);
fbackcontrol = controller();
trajectory_follower = trajectoryFollower(fcontrol, rtrajectory, fbackcontrol);

lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);