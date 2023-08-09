% Define a function to calculate the action vector for obstacle
function actionVector2 = calculateActionVector1_2(position, xO, yO, r2, s2, k2)
x = position(1);
y = position(2);
% dist = [x y; xO yO];

d2 = sqrt((x-xO)^2 + (y - yO)^2);
angle2 = atan2(yO - y, xO - x);
if d2 < r2
actionVector2(1) = -sign(cos(angle2))*120;
actionVector2(2) = -sign(sin(angle2))*120;
elseif r2 <= d2 && d2 <= s2 + r2
actionVector2 = -k2 * (s2+r2-d2) * [cos(angle2), sin(angle2)];
else
actionVector2 = [0, 0];
end
end

