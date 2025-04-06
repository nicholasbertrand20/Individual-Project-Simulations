clc; clear; close all;

% Parameters
R = 500; % Radius of the network area (meters)
lambda_user = 9/10000; % Density of users (users per m^2)

% Number of base stations and users
N_users = poissrnd(lambda_user * pi * R^2);  

% Generate base stations
x_BS = 0;
y_BS = 0;

% Generate random polar coordinates for users
theta_users = 2 * pi * rand(N_users, 1);
r_users = R * sqrt(rand(N_users, 1));
x_users = r_users .* cos(theta_users);
y_users = r_users .* sin(theta_users);

% Plot the Layout
figure;
hold on;
scatter(x_users, y_users, 10, 'r', 'filled'); % Users in red
scatter(x_BS, y_BS, 50, 'b', 'filled'); % Base stations in blue
legend('Interferers', 'BS');
xlabel('x (m)');
ylabel('y (m)');
grid on;
axis equal; % Keep aspect ratio square
axis([-500 500 -500 500]); % Set fixed axis limits
hold off;
