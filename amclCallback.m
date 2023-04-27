function amclCallback(msg)
    % Access the pose data from the message
    positionX = msg.Pose.Pose.Position.X+(mapWidth/2);
    positionY = msg.Pose.Pose.Position.Y+(mapHeight/2);

    % Update robot_pos matrix
    robot_pos = [positionX; positionY];
end