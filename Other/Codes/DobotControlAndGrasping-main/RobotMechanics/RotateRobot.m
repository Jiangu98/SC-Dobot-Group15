function RotateRobot(robot, angle)
    %Function that animates a Dobot rotating it's joint 1
    %Takes a Dobot handle and the desired angle
    % angle needs to be -135 -> 135 degrees

    steps = 100;

    qCurrent = robot.model.getpos();
    qTarget = qCurrent;
    qTarget(1) = deg2rad(angle);

    %Find q trajectory
    qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

    for i = 1:steps
        robot.model.animate(qTrajectory(i, :));
        drawnow();
    end

end
