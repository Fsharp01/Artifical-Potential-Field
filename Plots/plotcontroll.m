
function plotcontroll(target_points,x_list,y_list,steering_angle_list,delta_list)
plot(target_points(:, 1), target_points(:, 2), 'r*');
% Plot the new path
plot(x_list, y_list, 'b-');

% Plot the steering angle and delta
figure;
subplot(2, 1, 1);
plot(steering_angle_list);
xlabel('Iteration');
ylabel('Steering Angle');


subplot(2, 1, 2);
plot(delta_list);
xlabel('Iteration');
ylabel('Delta');
end

