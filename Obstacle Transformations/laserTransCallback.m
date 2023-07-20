function laserTransCallback(~,msg)
global startposx
global startposy
x=startposx;
y=startposy;
% Step 2: Obtain laser scan data using lidarScan
scan = msg;
plot(scan)

% Set the map resolution
mapResolution = 0.1;

% Compute the Cartesian coordinates (x, y) for each range measurement
angles = scan.angle_min:scan.angle_increment:scan.angle_max;
ranges = scan.ranges;

x = ranges .* cos(angles);
y = ranges .* sin(angles);

% Adjust the coordinates to match the desired coordinate frame
x = -x; % Invert the x-coordinate to start from the left
y = -y; % Invert the y-coordinate to start from the bottom




% % Step 3: Convert scan data to Cartesian coordinates
% cartesianPoints = scan.Cartesian;
% 
% % Step 4: Transform Cartesian coordinates to global coordinates using robot's pose
% transformedPoints = bsxfun(@plus, [cos(theta), -sin(theta), x; sin(theta), cos(theta), y; 0, 0, 1] * [cartesianPoints.'; ones(1, size(cartesianPoints, 1))]);
% 
% % Step 5: Convert transformed coordinates to occupancy grid indices
% indices = ceil(transformedPoints(1:2, :) / 0.1);
% 
% % Step 6: Update the occupancy map with occupied cells
% for i = 1:size(indices, 2)
%     if indices(1, i) >= 1 && indices(1, i) <= mapSize(2) && indices(2, i) >= 1 && indices(2, i) <= mapSize(1)
%         GlobaloccupancyMap(indices(2, i), indices(1, i)) = 100;  % Mark cell as occupied
%     end
% 
%     cmap = [1 1 1; 0 0 0; 0.5 0.5 0.5];
%     imagesc(x(:), y(:), GlobaloccupancyMap(:,:));
%     colormap(cmap);
%     colorbar;
%     axis equal;
%     xlabel('X');
%     ylabel('Y');
%     title('Global Occupancy Map');
% 
%     caxis([-1 100]);
%     ticks = linspace(-1,100,6);
%     labels = {'Unknown', 'Free', '', '', '', 'Occupied'};
%     colorbar('Ticks',ticks,'TickLabels',labels);
%     hold on
% end