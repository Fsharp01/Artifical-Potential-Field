function actionVector3 = calculateActionVector3(position, xO2, yO2, r3, s3, k3)
x = position(1);
y = position(2);
d3 = sqrt((x-xO2)^2 + (y - yO2)^2);
angle2 = atan2(yO2 - y, xO2 - x);
if d3 < r3
actionVector3(1) = -sign(cos(angle2))*80;
actionVector3(2) = -sign(sin(angle2))*80;
elseif  all([r3 <= d3, d3 <= s3 + r3])
actionVector3 = -k3 * (s3+r3-d3) * [cos(angle2), sin(angle2)];
else
actionVector3 = [0, 0];
end
end