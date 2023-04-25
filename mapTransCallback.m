function mapTransCallback(msg)
%load('globalmap.mat');
% global global2d=occupancyMap(globalmap);
local_map = rosReadOccupancyGrid(msg);
global local2d=occupancyMatrix(local_map);

    
%show(fixmap);
%title('Global Map')

show(local_map)
title('Real Time map')
hold on


%syncWith(globalmap, local_map);
%show(globalmap)
end
