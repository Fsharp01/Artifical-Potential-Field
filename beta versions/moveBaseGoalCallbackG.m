function moveBaseGoalCallbackG(~, msg3)
    % Define Goal
    goalPositionX = msg3.Pose.Position.X;
    goalPositionY = msg3.Pose.Position.Y;
    goal = [goalPositionX, goalPositionY];
    
    % Define obstacle positions and radii
    obstacle1 = [-9.945, 4.914];
    obstacle2 = [5.125, -5.039];
    obstacles = [obstacle1; obstacle2];
    obstacleRadii = [1; 1];
    
    % Define parameters
    radiusGoal = 1;
    spreadGoal = 26;
    constantGoal = 5;
    radiusObstacle = 1;
    spreadObstacle = 7;
    constantObstacle = -0.5;
    stepSize = 0.05;
    maxSpeed = 0.1;
    safetyRadius = 3;
    dt = 0.1;
    maxIterations = 1000;
    tolerance = 0.1;
    lookaheadDistance = 2;
    L = 0.25;  % Wheelbase length
    maxSteeringAngle = pi/4;  % Maximum steering angle limit
    
   global startposx
   global startposy
   global X
    global Y
    global occupancyMap
    global mapWidth 
    global mapHeight
    
    % Preallocate arrays
    Vx_G = zeros(size(X));
    Vy_G = zeros(size(Y));
    Vx_O = zeros(size(X));
    Vy_O = zeros(size(Y));
    
    % Calculate action vectors for goal and obstacles
    for i = 1:numel(X)
        position = [X(i), Y(i)];
        actionVector_G = calculateActionVector(position, goal, radiusGoal, spreadGoal, constantGoal);
        actionVector_O = calculateActionVector(position, obstacles, obstacleRadii, spreadObstacle, constantObstacle);
        Vx_G(i) = actionVector_G(1);
        Vy_G(i) = actionVector_G(2);
        Vx_O(i) = sum(actionVector_O(:, 1));
        Vy_O(i) = sum(actionVector_O(:, 2));
    end
    
    % Combine action vectors
    Vx = Vx_G + Vx_O;
    Vy = Vy_G + Vy_O;
    
% Define initial position and velocity
x1 = startposx;
y1 = startposy;
dx = 0;
dy = 0;
x_prev = x1;
y_prev = y1;

% Preallocate arrays for path and control
path = zeros(maxIterations, 2);
steering_angle_list = zeros(maxIterations, 1);
delta_list = zeros(maxIterations, 1);
target_points = zeros(maxIterations, 2);

% Run the simulation
for iteration = 1:maxIterations
    % Skip the first iteration (start position)
    if iteration > 1
        % Calculate the index of the grid point that the robot is currently on
        [~, idx] = pdist2([X(:), Y(:)], [x1, y1], 'euclidean', 'Smallest', 1);

        % Check if the robot has reached the goal
        if norm([x1 - goalPositionX, y1 - goalPositionY]) < radiusGoal
            fprintf('Goal reached after %d iterations\n', iteration);
            break;
        end
        
        % Calculate the action vector at the robot's current position
        actionVector = [Vx(idx), Vy(idx)];
        
        % Calculate the new position and velocity of the robot
        dx = dx + actionVector(1) * dt;
        dy = dy + actionVector(2) * dt;
        x_prev = x1;
        y_prev = y1;
        x1 = x1 + dx * dt;
        y1 = y1 + dy * dt;
        
        % Adjust the step size based on the distance traveled
        dist_traveled = norm([x1 - x_prev, y1 - y_prev]);
        if dist_traveled > stepSize
            dx = stepSize * dx / dist_traveled;
            dy = stepSize * dy / dist_traveled;
            x1 = x_prev + stepSize * dx / dist_traveled;
            y1 = y_prev + stepSize * dy / dist_traveled;
        end
        
        % Check if the robot has moved outside the field
        if x1 < X(1) || x1 > X(end) || y1 < Y(1) || y1 > Y(end)
            fprintf('Robot moved outside the field after %d iterations\n', iteration);
            break;
        end
        
        % Check for obstacle collision
        for i = 1:size(obstacles, 1)
            obstacle = obstacles(i, :);
            radiusObstacle = obstacleRadii(i);
            d_obstacle = norm([x1, y1] - obstacle) - radiusObstacle;
            if d_obstacle < safetyRadius
                % Adjust the velocity vector to steer away from the obstacle
                theta = atan2(y1 - obstacle(2), x1 - obstacle(1));
                dx = maxSpeed * cos(theta + pi / 2);
                dy = maxSpeed * sin(theta + pi / 2);
                x1 = x1 + dx * dt;
                y1 = y1 + dy * dt;
                if x1 < X(1) || x1 > X(end) || y1 < Y(1) || y1 > Y(end)
                    fprintf('Robot moved outside the field after %d iterations\n', iteration);
                    break;
                end
            end
        end
        
        % Store the robot's current position in the path array
        path(iteration, :) = [x1, y1];
        
        % Check if the robot has reached the goal
        if norm([x1, y1] - goal) < radiusGoal
            fprintf('Goal reached after %d iterations\n', iteration);
            break;
        end
        
        % Pause for a short time to allow for visualization
        pause(0.01);
    end
    
    % Plot the results
%     plotpathInit(path, x1, y1);
%     plotmovingPath(x1, y1, goalPositionX, goalPositionY, obstacle1(1), obstacle1(2), obstacle2(1), obstacle2(2), x_prev, y_prev, path, X, Y, Vx, Vy);
%     plotcontroll(target_points, path(:, 1), path(:, 2), steering_angle_list, delta_list);
end
% Initilaize Path size
load blankPoseMsg-1;
load blankPathMsg-1;
maxSize = size(path,1);

blankPoseMsgArray = repmat(blankPoseMsg,maxSize,1);
blankPathMsg.Poses = blankPoseMsgArray;

    %% Load Path data
pubPath = rospublisher("matlab_path","nav_msgs/Path", "DataFormat","struct");
pathMsg = blankPathMsg;

% Create a publisher for the '/cmd_vel' topic
pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
% Create a Twist message
twistMsg = rosmessage('geometry_msgs/Twist');
twist_msg = rosmessage('geometry_msgs/Twist');


pathMsg.Header.Seq = uint32(1);
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



    
end

function actionVector = calculateActionVector(position, obstacles, radii, spread, constant)
    x = position(1);
    y = position(2);
    d = sqrt((x - obstacles(:, 1)).^2 + (y - obstacles(:, 2)).^2);
    angles = atan2(obstacles(:, 2) - y, obstacles(:, 1) - x);
    actionVector = zeros(size(position));
    
    for i = 1:size(obstacles, 1)
        inRange = (radii(i) <= d) & (d <= spread + radii(i));
        actionVector(1) = actionVector(1) + sum(constant * (spread + radii(i) - d(inRange)) .* cos(angles(inRange)));
        actionVector(2) = actionVector(2) + sum(constant * (spread + radii(i) - d(inRange)) .* sin(angles(inRange)));
    end
end

function plotpathInit(path, x, y)
    figure;
    hold on;
    plot(path(:, 1), path(:, 2), 'k--');
    plot(x, y, 'ro');
end

function plotmovingPath(x, y, goalX, goalY, obstacle1X, obstacle1Y, obstacle2X, obstacle2Y, prevX, prevY, path, X, Y, Vx, Vy)
    plot(x, y, 'bo');
    hold on;
    plot(goalX, goalY, 'g*');
    plot(obstacle1X, obstacle1Y, 'r');
    plot(obstacle2X, obstacle2Y, 'r');
    plot([prevX, x], [prevY, y], 'b--');
    quiver(X, Y, Vx, Vy);
    plot(path(:, 1), path(:, 2), 'b-', 'LineWidth', 2);
    hold off;
    axis equal;
    axis([-15, 15, -15, 15]);
    drawnow;
end

function plotcontroll(targetPoints, x, y, steeringAngleList, deltaList)
    plot(targetPoints(:, 1), targetPoints(:, 2), 'r*');
    plot(x, y, 'b-');
    
    figure;
    subplot(2, 1, 1);
    plot(steeringAngleList);
    xlabel('Iteration');
    ylabel('Steering Angle');
    
    subplot(2, 1, 2);
    plot(deltaList);
    xlabel('Iteration');
    ylabel('Delta');
end