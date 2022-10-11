function PositionObject(robot, placementLocation, objectType)
    %Uses RMRC to place an object of the desired type at a specified location
    % Robot: desired robot to pickup object
    % ObjectType: type of desired object to place
    % placementLocation: location to place object
    startPoint = robot.model.fkine(robot.model.getpos());
    startPoint = startPoint(1:3, 4);
    endPoint = placementLocation;
    endPoint(3) = endPoint(3) + 0.02;

    RMRC(endPoint, 1, robot);

    switch objectType
        case 'strawberry'
            Strawberry(placementLocation);
        case 'grape'
            Grape(placementLocation);
        case 'pill'
            Pill(placementLocation);
        case 'lego'
            Lego(placementLocation);
    end

    RMRC(startPoint, 1, robot);
end
