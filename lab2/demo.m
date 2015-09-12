robot.startLaser();
pause(3);
laserScan = rossubscriber('/scan', {@laser_callback,robot});