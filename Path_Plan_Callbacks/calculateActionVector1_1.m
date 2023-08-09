% Define a function to calculate the action vector for goal
function actionVector = calculateActionVector1_1(position, xG, yG, r, s, k)

x = position(1);
y = position(2);
% 
% % Calculate the Euclidean distance between the two points
d = sqrt((x - xG)^2 + (y - yG)^2);
angle = atan2(yG - y, xG - x);
if d < r
actionVector = [0, 0];
elseif r <= d && d <= s + r
actionVector = k * (d - r) * [cos(angle), sin(angle)];
else
actionVector = k * s * [cos(angle), sin(angle)];
end
end


