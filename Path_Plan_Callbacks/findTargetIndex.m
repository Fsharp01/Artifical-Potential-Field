
function target_index = findTargetIndex(path, x2, y2, lookahead_distance, theta)
    target_index = 1;  % Initialize target index to the first point in the path

    % Iterate through each point in the path
    for i = 1:size(path, 1)
        % Calculate the distance from the robot to the current point in the path
        d = sqrt((path(i, 1) - x2)^2 + (path(i, 2) - y2)^2);

        % Check if the current point is ahead of the robot and within the lookahead distance
        if d > lookahead_distance && dot([path(i, 1) - x2, path(i, 2) - y2], [cos(theta), sin(theta)]) > 0
            % Update the target index to the current point in the path
            target_index = i;
            break;
        end
    end

end