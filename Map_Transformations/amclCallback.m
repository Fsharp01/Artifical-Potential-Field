function amclCallback(~,msg)

   global startposx
   global startposy
    % Access the pose data from the message
    positionX = msg.Pose.Pose.Position.X;
    positionY = msg.Pose.Pose.Position.Y;
    theta = msg.Pose.Pose.Orientation.Z;


    % Update robot_pos matrix
     
    startposx=positionX;
    startposy=positionY
     
end