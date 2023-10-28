function mapTransCallback(~, message)
%Held for live SLAM costmap
   % Access the occupancy grid data and metadata
   global loaded
   global obstacle_coordinates
   global X
   global Y

   %load FixOccupancyMap.mat;
  
    occupancyGridData = message.Data;
    mapInfo = message.Info;
    
    % Extract the map dimensions from the metadata
    mapWidth = mapInfo.Width;
    mapHeight = mapInfo.Height;

%     update static map
%     x=X;
%     y=Y;    % shift the x and y arrays
%     occupancyMap = reshape(occupancyGridData, mapWidth, mapHeight)';
%     occupancyMap = flipud(occupancyMap);
%     newOccupanyMap = [occupancyMap, zeros(size(occupancyMap, 1), size(GlobaloccupancyMap, 2)-size(occupancyMap, 2)); zeros(size(GlobaloccupancyMap, 1)-size(occupancyMap, 1), size(GlobaloccupancyMap, 2))];
%   LiveMap=GlobaloccupancyMap+newOccupanyMap;

  
  
  %     for i = 1:size(occupancyMap(1))
%             for j = 1:size(occupancyMap(2))                            
%                 if GlobaloccupancyMap(i, j) == 0 && newOccupanyMap(i, j)==100
%                     LiveMap(i,j)= 100;
%                 end
%                 
%             end
    
    

% map plot
%     cmap = [1 1 1; 0 0 0; 0.5 0.5 0.5];
%     plotobs=1;
%     obstacle_coordinates= plotObstacles(LiveMap,plotobs);
% 
%     imagesc(x(:), y(:), LiveMap(:,:));
%     colormap(cmap);
%     colorbar;
%     axis equal;
%     xlabel('X');
%     ylabel('Y');
%     title('Live Occupancy Map');
% 
%     caxis([-1 100]);
%     ticks = linspace(-1,100,6);
%     labels = {'Unknown', 'Free', '', '', '', 'Occupied'};
%     colorbar('Ticks',ticks,'TickLabels',labels);
    
end
