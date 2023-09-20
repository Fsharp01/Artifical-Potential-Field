
clear all;
rosshutdown
%masterhost='http://192.168.32.129:1311';
masterhost='http://192.168.32.129:1311';

rosinit(masterhost)

%% global Variables
global x
global y
global X
global Y
global theta
global GoalpositionX
global GoalpositionY
global pub_path
global robot_pos
global GlobaloccupancyMap
global obstacle_coordinates
global pub
global twistMsg
global pubPath
global pathMsg



global blankPoseMsg
global blankPathMsg
global   mapWidth 
global   mapHeight
global startposx
global startposy
global path
global Controllplot
global PathnInitplot
global MovingPathplot
global path_ready_flag
global t

timerset=0;
%% Define the constans
%plots
 Controllplot=0;
 PathnInitplot=0;
 MovingPathplot=0;

step_size = 0.4;
%% Define a grid of points in the 2D space from the map info
%load mapInfo123_2.mat;
%load occupancyGridData123_2.mat;
load mapInfo.mat
load OccupancyGridData.mat

 mapWidth = mapInfo.Width;
 mapHeight = mapInfo.Height;
 resolution=0.1;
[X, Y, GlobaloccupancyMap] = generateOccupancyMap(mapInfo, occupancyGridData);


%% Define a grid of points in the 2D space


sub2 = rossubscriber('/agent1/pose/amcl', 'geometry_msgs/PoseWithCovarianceStamped',@amclCallback);
sub3 = rossubscriber('/move_base_simple/goal', 'geometry_msgs/PoseStamped', @moveBaseGoalCallback);
% sub_map= rossubscriber("/map", "nav_msgs/OccupancyGrid", @mapTransCallback);


 
        % Create a publisher for the '/cmd_vel' topic
    pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    % Create a Twist message
    twistMsg = rosmessage('geometry_msgs/Twist');
    pubPath = rospublisher("matlab_path","nav_msgs/Path", "DataFormat","struct");
    pathMsg = blankPathMsg;
%sub_laser=rossubscriber("/agent1/scan","sensor_msgs/LaserScan", @laserTransCallback);

% 
% % Set the period for sending messages (in seconds)
 period = 0.1; % Change this to your desired period
% 
% % Create a timer object
 t = timer('ExecutionMode', 'fixedRate', 'Period', period, ...
           'TimerFcn', "sendROSMessage()");

 





