function actionVector = calculateActionVector(position, GoalpositionX, GoalpositionY, r, s, k)
    x = position(1);
    y = position(2);
    d = sqrt((x - GoalpositionX)^2 + (y - GoalpositionY)^2);
    angle = atan2(GoalpositionY - y, GoalpositionX - x);
    if d < r
        actionVector = [0, 0];
    elseif all([r <= d, d <= s + r])
        actionVector = k * (d - r) * [cos(angle), sin(angle)];
    else
        actionVector = k * s * [cos(angle), sin(angle)];
    end
end

