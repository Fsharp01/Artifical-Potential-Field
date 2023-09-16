function sendROSMessage(~)
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
    if pdist2([startposx, startposy], [Gx,Gy], 'euclidean') <= 0.225
     %if abs(startposx - Gx) <= 1 && abs(startposy - Gy) <= 1


           twistMsg.Linear.X = 0;

           twistMsg.Angular.Z = 0;
           path_ready_flag = false;
            send(pub, twistMsg);

    end
   
    % Publish the Twist message to the '/cmd_vel' topic
    send(pub, twistMsg);
    end
