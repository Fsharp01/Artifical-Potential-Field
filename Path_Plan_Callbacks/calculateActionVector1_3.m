% Define a function to calculate the action vector for obstacle2
function actionVector3 = calculateActionVector1_3(position, xO2, yO2, w2, h2, k3)
    x = position(1);
    y = position(2);
    
    left = xO2 - w2/2;
    right = xO2 + w2/2;
    top = yO2 + h2/2;
    bottom = yO2 - h2/2;
    
    if x >= left && x <= right && y >= bottom && y <= top
        if abs(x - left) < abs(x - right) % Closest to left side
            actionVector3 = [-k3, 0];
        elseif abs(x - left) > abs(x - right) % Closest to right side
            actionVector3 = [k3, 0];
        elseif abs(y - bottom) < abs(y - top) % Closest to bottom side
            actionVector3 = [0, -k3];
        else % Closest to top side
            actionVector3 = [0, k3];
        end
    else
        actionVector3 = [0, 0];
    end
end
