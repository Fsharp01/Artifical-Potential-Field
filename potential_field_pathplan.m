masterhost='http://losi-pc:11311';
rosinit(masterhost)

%% Create blank Pose message
load ./MAT/msg/blankPoseMsg blankPoseMsg;
load ./MAT/msg/blankPathMsg blankPathMsg;

%% Initilaize Path size
[~,maxSize] = size(x);
blankPoseMsgArray = repmat(blankPoseMsg,maxSize,1);
blankPathMsg.Poses = blankPoseMsgArray;

%% Define a grid of points in the 2D space
x = -15:0.5:15;
y = -15:0.5:15;
[X, Y] = meshgrid(x, y);


sub2 = rossubscriber('agent1/pose/amcl', 'geometry_msgs/PoseWithCovarianceStamped', @amclCallback);
sub3 = rossubscriber('/move_base_simple/goal', 'geometry_msgs/PoseStamped', @moveBaseGoalCallback);
pub_path = rospublisher("path","nav_msgs/Path", "DataFormat","struct");
pathMsg = blankPathMsg;


%% global Variables
global x
global y
global X
global Y
global GoalpositionX
global GoalpositionY
global pub_path
global robot_pos

global xO
global yO
global obstacle

global xO2
global yO2
global obstacle2


%% define the obstackle radius

global r
global r2
global r3

%% Define the field spread
global s
global s2
global s3

%% Define the constant
global k
global k2
global k3

global step_size
global position

global blankPoseMsg
global blankPathMsg



%% Define the starting position
robot_pos = [0, 0]; % initializes robot_pos to (0,0)

actionVector = [0, 0];
     
msg2 = receive(sub2);
amclCallback([], msg2);
disp(['Position: (' num2str(robot_pos(1)) ', ' num2str(robot_pos(2)) ')'])

%% Define the constans

%define the obstackles position

xO=-5;
yO=-5;
obstacle = [xO, yO];

xO2=5.5;
yO2=-9.5;
obstacle2 = [xO2, yO2];


%define the obstackle radius
r=1;
r2=3;
r3=3;

% Define the field spread
s=40;
s2=10;
s3=10;


% Define the constant
k=5;
k2=3;
k3=3;

step_size = 0.4;


% Calculate the action vectors for all points in the grid
Vx_G = zeros(size(X));
Vy_G = zeros(size(X));
Vx_O = zeros(size(X));
Vy_O = zeros(size(Y));
Vx_O2 = zeros(size(X));
Vy_O2= zeros(size(Y));

% Create empty arrays to store the robot's x and y positions
% x = robot_pos(1);
% y = robot_pos(2);

%% Calculating action vectors for all points 
for i = 1:numel(X)
    position = [X(i), Y(i)];


    actionVector = calculateActionVector(position, GoalpositionX, GoalpositionY, r, s, k);
    if size(actionVector, 2) == 0
    Vx_G(i) = 0;
    else
    Vx_G(i) = actionVector(1);
    end
    
    actionVector2 = calculateActionVector2(position, xO, yO, r2, s2, k2);
    Vx_O(i)=actionVector2(1); %obstacle
    Vy_O(i)=actionVector2(2);

    actionVector3 = calculateActionVector3(position, xO2, yO2, r3, s3, k3);
    Vx_O2(i)=actionVector3(1); %obstacle2
    Vy_O2(i)=actionVector3(2);

    Vx=Vx_G+Vx_O+Vx_O2;  %sum
    Vy=Vy_G+Vy_O+Vy_O2;
end
%% Plot the vectors using quiver
quiver(X, Y,Vx, Vy)

hold on

% Plot the path of the robot
plot(x, y, 'b', 'LineWidth', 1.5);
%% Callbacks
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

load blankPoseMsg;
load blankPathMsg;
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














