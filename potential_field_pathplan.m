clc;
clear all;
rosshutdown
masterhost='http://192.168.32.129:1311';
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



global blankPoseMsg
global blankPathMsg
global   mapWidth 
global   mapHeight
global startposx
global startposy
%% Define the constans



step_size = 0.4;
%% Define a grid of points in the 2D space from the map info
load mapInfo.mat;
load OccupancyGridData.mat;
 mapWidth = mapInfo.Width;
 mapHeight = mapInfo.Height;
 resolution=0.1;

[X, Y, occupancyMap] = generateOccupancyMap(mapInfo, occupancyGridData);




%% Define a grid of points in the 2D space



sub2 = rossubscriber('agent1/pose/amcl', 'geometry_msgs/PoseWithCovarianceStamped');

msg = receive(sub2);

% Access the pose data from the message
startposx = floor(msg.Pose.Pose.Position.X+(mapWidth/2)*resolution);
startposy = floor(msg.Pose.Pose.Position.Y+(mapHeight/2)*resolution);

% Print the position to the command window
%sub2 = rossubscriber('/agent1/pose/amcl', 'geometry_msgs/PoseWithCovarianceStamped');
%sub2.NewMessageFcn = @(~, msg) amclCallback(msg);
sub3 = rossubscriber('/move_base_simple/goal', 'geometry_msgs/PoseStamped', @moveBaseGoalCallback);
sub_map= rossubscriber("/map", "nav_msgs/OccupancyGrid", @mapTransCallback);


%% Define the starting position
 % initializes robot_pos to (0,0)


     
disp(['Position: (' num2str(startposx) ', ' num2str(startposy) ')'])













