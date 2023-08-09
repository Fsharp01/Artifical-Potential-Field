function amclCallback(~,msg)

   global startposx
   global startposy
   global path
   global d_last_target
   global lookahead_distance
   global target_index
   global path_ready_flag;
   global pub
   global theta
   global twistMsg
   global xG
   global yG
   
 
Gx=xG;
Gy=yG;

   target_index = 0;
   goal_reached = false;
   max_speed = 0.25;
    % Access the pose data from the message
    positionX = msg.Pose.Pose.Position.X;
    positionY = msg.Pose.Pose.Position.Y;
    quat=[msg.Pose.Pose.Orientation.X msg.Pose.Pose.Orientation.Y msg.Pose.Pose.Orientation.Z msg.Pose.Pose.Orientation.W];
    eulZYX  = quat2eul(quat);
    theta=eulZYX(3);


    % Update robot_pos matrix
     
    startposx=positionX;
    startposy=positionY;


   
    if path_ready_flag
             

         steering_angle = calculate_steering_angle(path, startposx, startposy, theta);
        
%     if d_last_target < lookahead_distance ||  target_index > size(path, 1)
%         goal_reached = true;
%         twistMsg.Linear.X = 0;
%         twistMsg.Angular.Z = 0;
%         path_ready_flag=false;
% 
%     end
    twistMsg.Linear.X = max_speed;
    twistMsg.Angular.Z = steering_angle;
       pdist2([startposx, startposy], [Gx,Gy], 'euclidean')
    if pdist2([startposx, startposy], [Gx,Gy], 'euclidean') <= 0.5
     %if abs(startposx - Gx) <= 1 && abs(startposy - Gy) <= 1


           twistMsg.Linear.X = 0;

           twistMsg.Angular.Z = 0;
           path_ready_flag = false;
            send(pub, twistMsg);

    end
   
    % Publish the Twist message to the '/cmd_vel' topic
    send(pub, twistMsg);
    end

end



