function amclCallback(~,msg)

   global startposx
   global startposy
   global path
   global d_last_target
   global lookahead_distance
   global target_index
   global path_ready_flag;
   global theta

   target_index = 0;
   goal_reached = false;
   max_speed = 1;
    % Access the pose data from the message
    positionX = msg.Pose.Pose.Position.X;
    positionY = msg.Pose.Pose.Position.Y;
    theta = msg.Pose.Pose.Orientation.Z;


    % Update robot_pos matrix
     
    startposx=positionX;
    startposy=positionY;

     % Wait until the path is ready 
    while ~path_ready_flag
        pause(0.1); 
    end

        
    if goal_reached
        return; % If the goal is reached, do nothing and return from the callback
    end

    steering_angle = calculate_steering_angle(path, startposx, startposy, theta);

    if d_last_target < lookahead_distance ||  target_index > size(path, 1)
        printf('Reached end of path in %d iterations\n', i);
        goal_reached = true;
    end

    twist_msg.Linear.X = max_speed;
    twistMsg.Angular.Z = steering_angle;
    
    % Publish the Twist message to the '/cmd_vel' topic
    send(pub, twistMsg);
     
end



