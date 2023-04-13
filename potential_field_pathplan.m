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

