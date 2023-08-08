% Define a function to calculate the action vector for obstacle2
function actionVector3 = calculateActionVector1_3(position, xO2, yO2, r3, s3, k3)
x = position(1);
y = position(2);
% dist = [x y; xO2 yO2];
% 
d3 = sqrt(double((x-xO2)^2) + double((y - yO2)^2));
angle2 = atan2(double(yO2 - y), double(xO2 - x));
if d3 < r3
actionVector3(1) = -sign(cos(angle2))*80;
actionVector3(2) = -sign(sin(angle2))*80;
elseif r3 <= d3 && d3 <= s3 + r3
actionVector3 = -k3 * (s3+r3-d3) * [cos(angle2), sin(angle2)];
else
actionVector3 = [0, 0];
end
end
