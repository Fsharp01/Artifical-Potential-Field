function plotmovingPath(x1,y1,xG,yG,xO,yO,xO2,yO2,x_prev,y_prev,path,X,Y,Vx,Vy)

plot(x1, y1, 'bo');
hold on;
plot(xG, yG, 'g*');
plot(xO, yO, 'r');
plot(xO2, yO2, 'r');
plot([x_prev, x1], [y_prev, y1], 'b--'); % plot a line connecting the previous position to the current position
quiver(X, Y, Vx, Vy);
plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
hold off;
axis equal;
axis([-15, 15, -15, 15]);
drawnow;
end