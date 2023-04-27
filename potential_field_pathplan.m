rosshutdown
masterhost='http://ubuntu:11311/';
rosinit(masterhost)

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
%% Define a grid of points in the 2D space from the map info
load mapInfo.mat;
load OccupancyGridData.mat;


[x, y, occupancyMap] = generateOccupancyMap(mapInfo, occupancyGridData);


% Calculate the action vectors for all points in the grid
Vx_G = zeros(size(x));
Vy_G = zeros(size(x));
Vx_O = zeros(size(x));
Vy_O = zeros(size(y));
Vx_O2 = zeros(size(x));
Vy_O2= zeros(size(y));


%% Define a grid of points in the 2D space



%sub2 = rossubscriber('agent1/pose/amcl', 'geometry_msgs/PoseWithCovarianceStamped', @amclCallback);
sub2 = rossubscriber('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped');
sub2.NewMessageFcn = @(~, msg) amclCallback(msg);
sub3 = rossubscriber('/move_base_simple/goal', 'geometry_msgs/PoseStamped', @moveBaseGoalCallback);
sub_map= rossubscriber("/map", "nav_msgs/OccupancyGrid", @mapTransCallback);


%% Define the starting position
 % initializes robot_pos to (0,0)

actionVector = [0, 0];
startposx = robot_pos(1);
startposy = robot_pos(2);
     
disp(['Position: (' num2str(startposx) ', ' num2str(startposy) ')'])





% Create empty arrays to store the robot's x and y positions
% x = robot_pos(1);
% y = robot_pos(2);
%% Calculating action vectors for all points 
for i = 1:numel(x)
    position = [x(i), y(i)];


    actionVector = calculateActionVector(position, GoalpositionX, GoalpositionY, r, s, k);
    Vx_G(i)=actionVector(1);
    Vy_G(i)=actionVector(2);
    
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
quiver(x, y,Vx, Vy)

hold on

% Plot the path of the robot
plot(x, y, 'b', 'LineWidth', 1.5);











