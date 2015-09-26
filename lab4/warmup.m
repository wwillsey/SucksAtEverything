global enc_init x_cmd ei tp;
enc_init = double(robot.encoders.LatestMessage.Left) / 1e3;
x_cmd = 1;
ei = 0;
tp;
error = -1
robot.sendVelocity(0.1, 0.1);
lh = event.listener(robot.encoders, 'OnMessageReceived', @PID);
tic;

while toc < 10
    pause(.001);
end
