function [w2, h2, w3, h3, w4, h4, w5, h5] = calculate_dimensions(corner_coordinates)
    % Initialize arrays to store dimensions
    widths = zeros(4, 1);
    heights = zeros(4, 1);

    for i = 1:4
        % Extract x and y coordinates
        xc = corner_coordinates(i, 1:2:end);
        yc = corner_coordinates(i, 2:2:end);
        
        % Calculate distances between adjacent corner points
        lengths = sqrt((xc(2) - xc(1))^2 + (yc(2) - yc(1))^2);
        widths(i) = sqrt((xc(3) - xc(1))^2 + (yc(3) - yc(1))^2);
        
        % Since it's an even rectangle, both lengths and widths are the same
        heights(i) = lengths;
    end

    % Assign the dimensions to respective variables
    w2 = widths(1);
    h2 = heights(1);
    w3 = widths(2);
    h3 = heights(2);
    w4 = widths(3);
    h4 = heights(3);
    w5 = widths(4);
    h5 = heights(4);
end
