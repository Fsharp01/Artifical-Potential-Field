function obstacleCentroids = findObstacleCentroids(occupancyMap)
    [rows, cols] = size(occupancyMap);
    visited = false(rows, cols);
    obstacleCentroids = [];
    
    for i = 6:rows
        for j = 6:cols
            if occupancyMap(i, j) > 0 && ~visited(i, j)
                centroid = findObstacleCentroid(occupancyMap, visited, i, j);
                obstacleCentroids = [obstacleCentroids; centroid];
            end
        end
    end
end