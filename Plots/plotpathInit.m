function plotpathInit(path,x2,y2)
figure;
hold on;
plot(path(:, 1), path(:, 2), 'k--')
plot(x2, y2, 'ro')
end