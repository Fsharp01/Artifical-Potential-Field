% Define a function to calculate the action vector for obstacle
function actionVector2 = calculateActionVector1_6(position, xO5, yO5, w5, h5, k3)
    x = position(1);
    y = position(2);
    
    left = xO5 - w5/2;
    right = xO5 + w5/2;
    top = yO5 + h5/2;
    bottom = yO5 - h5/2;
    
    if x >= left && x <= right && y >= bottom && y <= top
        if abs(x - left) < abs(x - right) % Closest to left side
            actionVector2 = [-k3, 0];
        elseif abs(x - left) > abs(x - right) % Closest to right side
            actionVector2 = [k3, 0];
        elseif abs(y - bottom) < abs(y - top) % Closest to bottom side
            actionVector2 = [0, -k3];
        else % Closest to top side
            actionVector2 = [0, k3];
        end
    else
        actionVector2 = [0, 0];
    end
end