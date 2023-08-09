function moveBaseGoalCallback(~, msg3)
%moveBaseGoalCallback(xgoal,ygoal)
   global startposx
   global startposy
   global path
   global path_ready_flag
   global xG
   global yG
   global pub_path
   global pathMsg
   global pubPath
   global Controllplot
   global PathnInitplot
   global MovingPathplot
   global X
   global Y
   global x
   global y
    global occupancyMap
    global mapWidth 
    global mapHeight
    global y2
    global x2
 path_ready_flag = false;
resolution=0.1;


%Define Goal
step_size = 0.06;
goal_treshold=0.5;
radius_G=0.1;
spread=26;
constant=8;
r = radius_G;
max_speed=0.2;

%define the obstackle position and radius

xO=-9.945;
yO=4.914;
obstacle = [xO, yO];
radius_O=1; %3
spread_O=10; %10
constant_O=-0.5;
r2 = radius_O;

%define the obstackle2 position and radius

xO2=5.125;
yO2=-5.039;
obstacle2 = [xO2, yO2];
radius_O2=1;
spread_O2=10;
constant_O2=-0.5;
r3 = radius_O2;

% Define the field spread
s = spread;
s2=spread_O;
s3=spread_O2;

% Define the constant
k = constant; %goal constant
k2=constant_O;
k3=constant_O2;



 % Access the position and orientation data from the message
    GoalpositionX = msg3.Pose.Position.X;
    GoalpositionY = msg3.Pose.Position.Y;
    xG=GoalpositionX;
    yG=GoalpositionY;
    goal = [GoalpositionX, GoalpositionY];
    disp(['Goal: (' num2str(GoalpositionX) ', ' num2str(GoalpositionY) ')']) 

    


% Calculate the action vectors for all points in the grid
Vx_G = zeros(size(X));
Vy_G = zeros(size(Y));
Vx_O = zeros(size(X));
Vy_O = zeros(size(Y));
Vx_O2 = zeros(size(X));
Vy_O2= zeros(size(Y));

% Create empty arrays to store the robot's x and y positions
x1 = startposx;
y1 = startposy;


for i = 1:numel(X)
    position = [X(i), Y(i)];
    actionVector = calculateActionVector1_1(position, xG, yG, r, s, k);
    Vx_G(i) = actionVector(1);  %goal
    Vy_G(i) = actionVector(2);
    actionVector2 = calculateActionVector1_2(position, xO, yO, r2, s2, k2);
    Vx_O(i)=actionVector2(1); %obstacle
    Vy_O(i)=actionVector2(2);

    actionVector3 = calculateActionVector1_3(position, xO2, yO2, r3, s3, k3);
    Vx_O2(i)=actionVector3(1); %obstacle2
    Vy_O2(i)=actionVector3(2);

    Vx=Vx_G+Vx_O+Vx_O2;  %sum
    Vy=Vy_G+Vy_O+Vy_O2;

end

% Combine the x and y action vectors into a single matrix
V = [Vx(:), Vy(:)];
% Define the initial position and velocity of the robot
dx = 0;
dy = 0;

% Define the variable to store the previous position
x_prev = x1;
y_prev = y1;

% Define empty arrays to store dx and dy at each time step
dx_array = [];
dy_array = [];

% Define the simulation parameters
max_iterations = 10000;
tolerance = 0.1;
dt = 0.1;
path = [];
safety_radius = 0.5;

% Run the simulation
for i = 1:max_iterations
    % Calculate the index of the grid point that the robot is currently on
    [d, idx] = pdist2([X(:), Y(:)], [x1, y1], 'euclidean', 'Smallest', 1);

    % Check if the robot has reached the goal %%%%%%%%%
if pdist2([x1, y1], goal, 'euclidean') < goal_treshold
    fprintf('Goal reached after %d iterations\n', i);
    break;
end

% Calculate the action vector at the robot's current position
actionVector = V(idx, :);
% Calculate the new position and velocity of the robot
dx = dx + actionVector(1)*dt;
dy = dy + actionVector(2)*dt;
x_prev = x1; % store the previous position
y_prev = y1;
x1 = x1 + dx*dt;
y1 = y1 + dy*dt;

 % Adjust the step size based on the distance traveled
  dist_traveled = norm([x1-x_prev, y1-y_prev]);
    if dist_traveled > step_size
        dx = step_size * dx / dist_traveled;
        dy = step_size * dy / dist_traveled;
        x1 = x_prev + step_size * dx / dist_traveled;
        y1 = y_prev + step_size * dy / dist_traveled;
    end

% Check if the robot has moved outside the field
if x1 < x1(1) || x1 > x1(end) || y1 < y1(1) || y1 > y1(end)
    fprintf('Robot moved outside the field after %d iterations\n', i);
    break;
end

 

    % Check for obstacle collision
    d_obstacle = pdist2([x1, y1], obstacle, 'euclidean') - r2;
    if d_obstacle < r2
        % Adjust the velocity vector to steer away from the obstacle
        theta = atan2(y1 - obstacle(2), x1 - obstacle(1));
        dx = max_speed * cos(theta + pi / 2);
        dy = max_speed * sin(theta + pi / 2);
        
        % Check if the robot is still within the safety distance
        while d_obstacle < safety_radius
            % Move the robot away from the obstacle
            x1 = x1 + dx * dt;
            y1 = y1 + dy * dt;
            
            % Update the distance to the obstacle
            d_obstacle = pdist2([x1, y1], obstacle, 'euclidean') - safety_radius;
            
            % Check if the robot has moved outside the field
            if x1 < X(1) || x1 > X(end) || y1 < Y(1) || y1 > Y(end)
                fprintf('Robot moved outside the field after %d iterations\n', i);
                break;
            end
        end
    end
    
    % Check for obstacle2 collision
d_obstacle2 = pdist2([x1, y1], obstacle2, 'euclidean') - safety_radius;
if d_obstacle2 < r3
    % Adjust the velocity vector to steer away from the obstacle
    theta = atan2(y1 - obstacle2(2), x1 - obstacle2(1));
    dx = max_speed * cos(theta + pi / 2);
    dy = max_speed * sin(theta + pi / 2);

    while d_obstacle2 < safety_radius
        % Move the robot away from obstacle2
        x1 = x1 + dx * dt;
        y1 = y1 + dy * dt;

        % Update the distance to obstacle2
        d_obstacle2 = pdist2([x1, y1], obstacle2, 'euclidean') - safety_radius;

        % Check if the robot has moved outside the field
        if x1 < X(1) || x1 > X(end) || y1 < Y(1) || y1 > Y(end) %itt
            fprintf('Robot moved outside the field after %d iterations\n', i);
            break;
        end
    end
end

% Store the robot's current position in the path array
path(end+1, :) = [x1, y1];

% Plot the robot's position on the field
if MovingPathplot==1
plotmovingPath(x1,y1,xG,yG,xO,yO,xO2,yO2,x_prev,y_prev,path,X,Y,Vx,Vy);
end


% Check if the robot has reached the goal
if abs(x1 - xG) <= 0.1 && abs(y1 - yG) <= 0.1
    fprintf('Goal reached after %d iterations\n', i);
    break;
end

% Pause for a short time to allow for visualization
pause(0.01);
end



load blankPoseMsg-1;
load blankPathMsg-1;

%% Initilaize Path size
maxSize = size(path,1);

blankPoseMsgArray = repmat(blankPoseMsg,maxSize,1);
blankPathMsg.Poses = blankPoseMsgArray;

    %% Load Path data
pubPath = rospublisher("matlab_path","nav_msgs/Path", "DataFormat","struct");
pathMsg = blankPathMsg;

pathMsg.Header.Seq = uint32(1);
pathMsg.Header.FrameId = 'map';
for i=1:maxSize
    pathMsg.Header.Seq = pathMsg.Header.Seq + 1;
    pathMsg.Poses(i).Pose.Position.X = path(i,1);
    pathMsg.Poses(i).Pose.Position.Y = path(i,2);
    pathMsg.Poses(i).Pose.Position.Z = 0;
    pathMsg.Poses(i).Pose.Orientation.X = 0;
    pathMsg.Poses(i).Pose.Orientation.Y = 0;
    pathMsg.Poses(i).Pose.Orientation.Z = 0;
    pathMsg.Poses(i).Pose.Orientation.W = 1;
    pathMsg.Poses(i).Header.FrameId = 'map';
end
pathMsg.Header.FrameId = 'map';

send(pubPath,pathMsg);
path_ready_flag = true;




    
end





