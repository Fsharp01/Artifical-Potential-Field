function centroid = findObstacleCentroid(occupancyMap, visited, startRow, startCol)
    [rows, cols] = size(occupancyMap);
    queue = [startRow, startCol];
    visited(startRow, startCol) = true;
    numPoints = 1;
    sumRow = startRow;
    sumCol = startCol;

    while ~isempty(queue)
        current = queue(1, :);
        queue(1, :) = [];
        row = current(1);
        col = current(2);
        
        for i = max(1, row-1):min(rows, row+1)
            for j = max(1, col-1):min(cols, col+1)
                if occupancyMap(i, j) > 0 && ~visited(i, j)
                    queue = [queue; i, j];
                    visited(i, j) = true;
                    numPoints = numPoints + 1;
                    sumRow = sumRow + i;
                    sumCol = sumCol + j;
                end
            end
        end
    end

    centroid = [sumRow/numPoints, sumCol/numPoints];
end
