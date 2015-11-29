global robot
fh = figure(1);
d = robotKeypressDriver(fh);
while true
    robotKeypressDriver.drive(robot,1)
    pause(.1);
end