function moveBaseGoalCallback(~, msg3)
    global pub_path
    global GoalpositionX
    global GoalpositionY
    global X
    global Y
    global x
    global y
    global xO
global yO
global obstacle

global xO2
global yO2
global obstacle2


%define the obstackle radius
global r
global r2
global r3

% Define the field spread
global s
global s2
global s3

% Define the constant
global k
global k2
global k3
global blankPoseMsg
global blankPathMsg
global position
global robot_pos


step_size=0.5;



    % Access the position and orientation data from the message
    GoalpositionX = msg3.Pose.Position.X/0.1;
    GoalpositionY = msg3.Pose.Position.Y/0.1;
    goal = [GoalpositionX, GoalpositionY];
    disp(['Goal: (' num2str(GoalpositionX) ', ' num2str(GoalpositionY) ')']) 

    

    % tervezés
    for i = 1:numel(X)

   
    % Calculate the action vector for the robot's current position
    actionVector4 = calculateActionVector(robot_pos,GoalpositionX, GoalpositionY, r, s, k) ...
        + calculateActionVector2(robot_pos,  xO, yO, r2, s2, k2) ...
        + calculateActionVector3(robot_pos,  xO2, yO2, r3, s3, k3);

    % Normalize the action vector to obtain a unit vector in the direction of motion
    unit_vector = actionVector4 / norm(actionVector4);
    
    % Move the robot in the direction of the action vector at the fixed step size
    new_pos = robot_pos + step_size .* unit_vector;

      % Check if the robot has reached the goal position
    if norm(new_pos - goal) < 0.01
        disp("Robot reached goal!");
        break;
    end


% Define the safety distance from the obstacles
safety_dist = 0.5;

% Check if the robot collides with an obstacle
if norm(new_pos - obstacle) < r2 + safety_dist
    % Compute the vector from the obstacle to the robot
    vector_to_robot = robot_pos - obstacle;
    % Compute the tangent vector to the obstacle
    tangent_vector = [-vector_to_robot(2); vector_to_robot(1)];
    % Compute the component of the action vector that's parallel to the tangent vector
    parallel_vector = dot(unit_vector, tangent_vector) * tangent_vector;
    % Compute the component of the action vector that's orthogonal to the tangent vector
    orthogonal_vector = unit_vector - parallel_vector;
    % Compute the new action vector by reversing the orthogonal vector and adding it to the parallel vector
    new_unit_vector = parallel_vector - orthogonal_vector;
    % Move the robot in the direction of the new action vector
    robot_pos = robot_pos + step_size * new_unit_vector;
elseif norm(new_pos - obstacle2) < r3 + safety_dist
    % Compute the vector from the obstacle to the robot
    vector_to_robot = robot_pos - obstacle2;
    % Compute the tangent vector to the obstacle
    tangent_vector = [-vector_to_robot(2); vector_to_robot(1)];
    % Compute the component of the action vector that's parallel to the tangent vector
    parallel_vector = dot(unit_vector, tangent_vector) * tangent_vector;
    % Compute the component of the action vector that's orthogonal to the tangent vector
    orthogonal_vector = unit_vector - parallel_vector;
    % Compute the new action vector by reversing the orthogonal vector and adding it to the parallel vector
    new_unit_vector = parallel_vector - orthogonal_vector;
    % Move the robot in the direction of the new action vector
    robot_pos = robot_pos + step_size * new_unit_vector;
else
    % Move the robot to the new position
    robot_pos = new_pos;
end

       % Append the robot's x and y positions to the arrays
    x = [x, robot_pos(1)];
    y = [y, robot_pos(2)];
  
    end

load blankPoseMsg-1;
load blankPathMsg-1;

%% Initilaize Path size
[~,maxSize] = size(x);
blankPoseMsgArray = repmat(blankPoseMsg,maxSize,1);
blankPathMsg.Poses = blankPoseMsgArray;

    %% Load Path data
pubPath = rospublisher("path","nav_msgs/Path", "DataFormat","struct");
pathMsg = blankPathMsg;




pathMsg.Header.Seq = uint32(1);
for i=1:maxSize
    pathMsg.Header.Seq = pathMsg.Header.Seq + 1;
    pathMsg.Poses(i).Pose.Position.X = x(i)*0.1;
    pathMsg.Poses(i).Pose.Position.Y = y(i)*0.1;
    pathMsg.Poses(i).Pose.Position.Z = 0;
    pathMsg.Poses(i).Pose.Orientation.X = 0;
    pathMsg.Poses(i).Pose.Orientation.Y = 0;
    pathMsg.Poses(i).Pose.Orientation.Z = 0;
    pathMsg.Poses(i).Pose.Orientation.W = 1;
    pathMsg.Poses(i).Header.FrameId = 'map';
end
pathMsg.Header.FrameId = 'map';
%[pubPath,pathMsg] = pathPublisher(x,y);

send(pubPath,pathMsg);
    
end

