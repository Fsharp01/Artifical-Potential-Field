function mapTransCallback(~, message)
%Held for live SLAM costmap
   % Access the occupancy grid data and metadata
    occupancyGridData = message.Data;
    mapInfo = message.Info;
    
    % Extract the map dimensions from the metadata
    mapWidth = mapInfo.Width;
    mapHeight = mapInfo.Height;
    
    % Convert the occupancy grid data to a 2D matrix
    map = reshape(occupancyGridData, mapWidth, mapHeight)';
    
    % Create the x and y matrices
     [x,y] = meshgrid(1:mapWidth, 1:mapHeight);
    
    % Example: plot the occupancy grid data with x and y coordinates
    figure(1)
    surf(x,y,map);
    colormap('gray')
    axis equal
end
