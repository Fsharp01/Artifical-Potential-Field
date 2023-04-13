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