function GrabAndReturn(robot, object)
    %Uses RMRC to move robot to the object and then return to it's starting position
    % Robot: desired robot to pickup object
    % Object: desired object to move

    midPoint = object.location;
    midPoint(3) = midPoint(3) + 0.02;

    endPoint = midPoint;
    endPoint(3) = endPoint(3) + 0.1;

    RMRC(midPoint, 1, robot);
    try delete(object); end
    RMRC(endPoint, 1, robot);
end
