global robot
for i = 1 : 10
    ranges = robot.laser.LatestMessage.Ranges;
    name = ['lab8/range_image_' int2str(i)];
    save(name, 'ranges');
    pause(10);
    disp('beep');
end