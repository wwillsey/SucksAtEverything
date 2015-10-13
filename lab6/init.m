close all;
global robot_trajectory1 robot_trajectory2 robot_trajectory3;
global fbackcontrol trajectory_follower ;
global t_accum tp running;
global error_plot feed_back_plot;
global terminate;
global current;
global feed_back_plot1 feed_back_plot2 feed_back_plot3;

figure(2)
f2 = plot(0,0,0,0);
xlabel('x');
ylabel('y');
legend('feedback', 'reference');
figure(3)
f3 = plot(0,0,0,0);
xlabel('x');
ylabel('y');
legend('feedback', 'reference');
figure(4)
f4 = plot(0,0,0,0);
xlabel('x');
ylabel('y');
legend('feedback', 'reference');
feed_back_plot1 = plotter(f2);
feed_back_plot2 = plotter(f3);
feed_back_plot3 = plotter(f4);

current = 1;
terminate = false;
robot_trajectory1 = cubicSpiral.planTrajectory(0.25, 0.25, 0, 1);
robot_trajectory1.planVelocities(0.25);
robot_trajectory2 = cubicSpiral.planTrajectory(-0.5, -0.5, -pi/2, 1);
robot_trajectory2.planVelocities(0.25);
robot_trajectory3 = cubicSpiral.planTrajectory(-0.25, 0.25, pi/2, 1);
robot_trajectory3.planVelocities(0.25);
fbackcontrol = controller();
trajectory_follower = trajectoryFollower(robot_trajectory1, fbackcontrol);
global_reset;
running = true;
lh = event.listener(robot.encoders, 'OnMessageReceived', @encoder_callback);
%enc = rossubscriber('/enc', @encoder_callback);