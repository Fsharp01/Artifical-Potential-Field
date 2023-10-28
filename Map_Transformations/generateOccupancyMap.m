function [x, y, occupancyMap] = generateOccupancyMap(mapInfo, occupancyGridData)
global simulation
    resolution =0.1;
    mapWidth = mapInfo.Width;
    mapHeight = mapInfo.Height;

 %   [x,y] = meshgrid(1:mapWidth, 1:mapHeight);
  %  occupancyMap = reshape(occupancyGridData, mapWidth, mapHeight)';
   % occupancyMap = flipud(occupancyMap);

  %original big [x,y] = meshgrid(((mapWidth/2-mapWidth)*resolution):(floor(mapWidth/2)*resolution)), ((mapHeight/2-mapHeight)*resolution):(floor(mapHeight/2)*resolution)); % shift the x and y arrays
 %13x15  x_coords = ((mapWidth/2-mapWidth)*resolution):(floor(mapWidth/2)*resolution);
%y_coords = ((mapHeight/2-mapHeight)*resolution):(floor(mapHeight/2)*resolution);
%[x, y] = meshgrid(x_coords, y_coords);

if simulation==0
xrange = -3:0.1:7;
yrange = 5.735:-0.1:-2.635;
end
if simulation==1
xrange = -13.7:0.4:27.4;
yrange = -12.2:0.4:24.4;
end
[x, y] = meshgrid(xrange, yrange);
x=double(x);
y=double(y);


   occupancyMap = reshape(occupancyGridData, mapWidth, mapHeight)';
    %occupancyMap = flipud(occupancyMap);
     
    cmap = [1 1 1; 0 0 0; 0.5 0.5 0.5];

    imagesc(x(:), y(:), occupancyMap(:,:));
    colormap(cmap);
    colorbar;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title('Global Occupancy Map');
    set(gca,'YtickLabel',5.735:-1:-2.635);
    caxis([-1 100]);
    ticks = linspace(-1,100,6);
    labels = {'Unknown', 'Free', '', '', '', 'Occupied'};
    colorbar('Ticks',ticks,'TickLabels',labels);
    hold on
% end
