function mapTransCallback(~, message)
%Held for live SLAM costmap
   % Access the occupancy grid data and metadata
    occupancyGridData = message.Data;
    mapInfo = message.Info;
    
    % Extract the map dimensions from the metadata
    mapWidth = mapInfo.Width;
    mapHeight = mapInfo.Height;
    
    [x,y] = meshgrid((1:mapWidth)-(mapWidth/2-275/2), (1:mapHeight)-(mapHeight/2-245/2)); % shift the x and y arrays
    occupancyMap = reshape(occupancyGridData, mapWidth, mapHeight)';
    occupancyMap = flipud(occupancyMap);

    cmap = [1 1 1; 0 0 0; 0.5 0.5 0.5];

    imagesc(x(:), y(:), occupancyMap(:,:));
    colormap(cmap);
    colorbar;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title('Live Occupancy Map');

    caxis([-1 100]);
    ticks = linspace(-1,100,6);
    labels = {'Unknown', 'Free', '', '', '', 'Occupied'};
    colorbar('Ticks',ticks,'TickLabels',labels);
end
