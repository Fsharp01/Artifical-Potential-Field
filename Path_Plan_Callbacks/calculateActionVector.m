
function actionVector = calculateActionVector(position, xG, yG, r, s, k)

x = position(1);
y = position(2);
d = sqrt(double((x - xG)^2) + double((y - yG)^2));
angle = atan2(double(yG - y), double(xG - x));
if d < r
actionVector = [0, 0];
elseif r <= d && d <= s + r
actionVector = k * (d - r) * [cos(angle), sin(angle)];
else
actionVector = k * s * [cos(angle), sin(angle)];
end
end