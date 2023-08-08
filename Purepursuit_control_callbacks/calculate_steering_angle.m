function steering_angle = calculate_steering_angle(path, startposx, startposy, theta)

global d_last_target
global lookahead_distance
global target_index
global theta
global startposx
global startposy
global path

% Create empty arrays to store the robot's x and y positions
x2 = startposx;
y2 = startposy;
% Create empty arrays to store the steering angle and delta
steering_angle_list = [];
delta_list = [];
% Create empty arrays to store the target points
target_points = [];
x_list = [x2];
y_list = [y2];
robot_position = [x2, y2];

% Define the lookahead distance
lookahead_distance = 2;
max_speed = 1;
dt=0.1;
max_iterations = 1000;
% Define the vehicle parameters
L = 0.25; % Wheelbase length
max_steering_angle = pi/4; % Maximum steering angle limit


% Get the last point on the path
last_point = path(end, :);


     target_index = findTargetIndex(path, x2, y2, lookahead_distance, theta);

    % Get the coordinates of the target point
    x_target = path(target_index, 1);
    y_target = path(target_index, 2);

    % Calculate the distance from the robot to the target point
    d_target = vecnorm([x_target - x2, y_target - y2], 2, 2);

 % Calculate the distance to the last target point
    d_last_target = norm([last_point(1) - x2, last_point(2) - y2]);


    % Calculate the new orientation
    alpha = atan2(y_target - y2, x_target - x2);

    % Calculate the desired curvature of the path
    %curvature = 2 * sin(alpha) / d_target;

    % Calculate the desired steering angle using the desired curvature
    desired_steering_angle = atan2(2*L*sin(alpha - theta), d_target);

    % Limit the steering angle within the maximum steering angle limit
    steering_angle = max(-max_steering_angle, min(max_steering_angle, desired_steering_angle));

    % Calculate the change in orientation
    theta_dot = max_speed * tan(steering_angle) / L;

    % Append new position to the lists
    x_list(end + 1) = x2;
    y_list(end + 1) = y2;

    % Append steering angle and delta to the lists
    steering_angle_list(end + 1) = desired_steering_angle;
    delta_list(end + 1) = theta_dot;

    % Return the steering angle
    steering_angle = steering_angle_list;
end
