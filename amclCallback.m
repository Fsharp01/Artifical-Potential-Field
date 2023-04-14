function amclCallback(~, msg2)
    global robot_pos
    global onceFlag
    
   
        % Access the pose data from the message
        positionX = msg2.Pose.Pose.Position.X;
        positionY = msg2.Pose.Pose.Position.Y;

        % Update robot_pos matrix
        robot_pos(1) = positionX;
        robot_pos(2) = positionY;

   
end