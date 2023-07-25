function amclCallback(~,msg)


    % Access the pose data from the message
    positionX = msg.Pose.Pose.Position.X;
    positionY = msg.Pose.Pose.Position.Y;

    % Update robot_pos matrix
     
    startposx=positionX;
    startposy=positionY
     
end