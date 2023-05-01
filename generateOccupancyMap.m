function [x, y, occupancyMap] = generateOccupancyMap(mapInfo, occupancyGridData)
    transformconst = mapInfo.Resolution;
    mapWidth = mapInfo.Width;
    mapHeight = mapInfo.Height;

 %   [x,y] = meshgrid(1:mapWidth, 1:mapHeight);
  %  occupancyMap = reshape(occupancyGridData, mapWidth, mapHeight)';
   % occupancyMap = flipud(occupancyMap);

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
    title('Global Occupancy Map');

    caxis([-1 100]);
    ticks = linspace(-1,100,6);
    labels = {'Unknown', 'Free', '', '', '', 'Occupied'};
    colorbar('Ticks',ticks,'TickLabels',labels);
    hold on
end
