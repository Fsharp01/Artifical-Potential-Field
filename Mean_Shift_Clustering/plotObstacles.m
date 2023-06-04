function ObstacleCoordinates = plotObstacles(occupancyMap,show)
%if plot=1 it will be plotted else not
    obstacleCentroids = findObstacleCentroids(occupancyMap);
    unique_pairs = unique(obstacleCentroids, 'rows');
    [rows, cols] = size(unique_pairs);
pointToRemove = [124 132];
  for i = 1:rows
        for j = 1:cols
            if floor(unique_pairs(i, j)) == pointToRemove(1, 1) || floor(unique_pairs(i, j)) == pointToRemove(1, 2)
                unique_pairs(i,j)=0;
            end
        end
  end
  ObstacleCoordinates=unique_pairs;
  if show==1
      figure;
    imagesc(occupancyMap);
    colormap([1 1 1; 1 0 0]);
    hold on;
    plot(unique_pairs(:, 2), unique_pairs(:, 1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    hold off;
  end

end