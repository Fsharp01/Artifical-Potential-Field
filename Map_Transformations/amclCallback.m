function amclCallback(~,msg)


    % Access the pose data from the message
    positionX = floor(msg.Pose.Pose.Position.X+(275/2));
    positionY = floor(msg.Pose.Pose.Position.Y+(245/2));

    % Update robot_pos matrix
     
    startposx=positionX;
    startposy=positionY
     
end