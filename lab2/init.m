% Node = robotics.ros.Node('nxt_velocity');
robot.startLaser();
%global pub;
%pub = rospublisher('/nxt_velocity', 'std_msgs/Float32MultiArray');
pause(3);
laserScan = rossubscriber('/scan', {@laser_callback, robot});
