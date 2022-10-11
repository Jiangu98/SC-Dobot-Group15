function AnimateRobots(robot, qTarget)
    %Function that animates a robos on a desired
    %trajectory. Takes a robot, it's current joint configuration
    %and the desired joint configuration.

    steps = 100;
    qCurrent = robot.model.getpos();

    %Find q trajectory
    qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

    for i = 1:steps
        robot.model.animate(qTrajectory(i, :));
        drawnow();
    end

end

% transformFruit = robot.model.fkine(qTrajectory(i, :));
% verticesTransformed = [fruit.vertices, ones(size(fruit.vertices, 1), 1)] * transformFruit';
% set(fruit.self, 'Vertices', verticesTransformed(:, 1:3));
