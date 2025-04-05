% --- This simulation is used to display Fig. 4.4a and 4.4b ---

clc; clear; close all;

% Initial Variables
Nmax = 10e5;                                    % Number of simulations
D_x = 47;                                       % Warehouse width (m)
D_y = 19;                                       % Warehouse depth (m)

% Rack dimensions (m)
s_c = 3;                                        
s_r = 3;
w_r = 3;
d_r = 7;

f_c = 2.8e9;                                    % Carrier freq (Hz)
c = 3e8;                                        % Speed of light (m/s)
lambda = c / f_c;                               % Wavelength (m)
gam_thr = 10^(3/10);                            % Gamma threshold 
h = 3;                                          % Height of building (m)
alpha_default = 0.01;                           % xi values
alpha_05 = 0.05;
noise_power = -90;                              % (dBm)
sigma_2 = 10^(noise_power/10) / 1000;           % Noise power (linear)

eta = lambda^2 / (16 * pi^2);

% Variable to be varied
gam_t = linspace(45, 70, 50);                   % Generates 50 points between the first two specified numbers
P_t = 10.^(gam_t/10) * sigma_2;                 % Power transmitted (W)

% Initialising variables
P_out = zeros(Nmax, length(gam_t));
P_out_30 = zeros(Nmax, length(gam_t));
P_out_30_10 = zeros(Nmax, length(gam_t));
P_out_05 = zeros(Nmax, length(gam_t));
P_out_30_05 = zeros(Nmax, length(gam_t));
P_out_30_05_10 = zeros(Nmax, length(gam_t));

% Calculate the Number of Aisles
num_aisles = floor(D_x / (w_r + s_c)); 

% Generate x-coordinates for PAs (center of aisles)
x_PA = s_c/2 : w_r + s_r : D_x - (s_c / 2);

N = length(x_PA);

for num = 1:Nmax
    
    % Generate random user's coordinates
    U_i_x = D_x * rand();
    U_i_y = D_y * rand();             % If the PA is in the middle row of the warehouse!
    
    % Distance between PA and UE if PA is in [x_PA, D_y/2, h]
    [UE_PA_dist, nearest_PA_idx] = min( sqrt( (U_i_x - x_PA).^2 + (U_i_y - D_y/2).^2 + h^2 ) );
    nearest_x_PA = x_PA(nearest_PA_idx); % Get the x-coordinate of the closest PA

    % For when xi = 0.01
    SINR = ( eta * N * P_t * exp(-alpha_default * nearest_x_PA) ) / ( sigma_2 * ( UE_PA_dist^2 ) );  % SNR 
    P_out(num,:) = SINR <= gam_thr; % Outage probability
    Rp = log2 ( 1 + SINR ); % Spectral Efficiency

    % For when xi = 0.05    
    SINR_05 = ( eta * N * P_t * exp(-alpha_05 * nearest_x_PA) ) / ( sigma_2 * ( UE_PA_dist^2 ) ); % SNR
    P_out_05(num,:) = SINR_05 <= gam_thr; % Outage probability
    Rp_05 = log2 ( 1 + SINR_05 ); % Spectral Efficiency


end

% Mean all simulations 
mean_P_out = max(mean(P_out, 1), 1e-6);
mean_P_out_05 = max(mean(P_out_05, 1), 1e-6);

mean_Rp = mean(Rp, 1);
mean_Rp_05 = mean(Rp_05, 1);

% Figure for Outage probability vs SNR
figure;
semilogy(gam_t, mean_P_out, '-x', 'LineWidth', 2, 'Color', 'r');
hold on;
semilogy(gam_t, mean_P_out_05, '-o', 'LineWidth', 2, 'Color', 'b');
xlabel('SNR (dB)', 'FontSize', 11);
ylabel('Outage Probability P_o_u_t', 'FontSize', 11);

ax = gca; % Get current axes
ax.FontSize = 12; % Set larger font size for x and y axis values (tick labels)
ax.TickDir = 'out'; % Set ticks to point outwards

legend("\xi = 0.01", "\xi = 0.05", 'FontSize', 20);
grid on;
ylim([1e-5 1]);  % Set Y-axis range from 10^-5 to 1     

% Figure for Spectral Efficiency vs SNR
figure;
plot(gam_t, mean_Rp, '-x', 'LineWidth', 2, 'Color', 'r');
hold on;
plot(gam_t, mean_Rp_05, '-o', 'MarkerSize', 2, 'LineWidth', 2, 'Color', 'b');
xlabel('SNR (dB)', 'FontSize', 11);
ylabel('Mean Spectral Efficiency', 'FontSize', 11);

lgd = legend('show');
legend("\xi = 0.01", "\xi = 0.05", 'FontSize', 20);

ax = gca; % Get current axes
ax.FontSize = 12; % Set larger font size for x and y axis values (tick labels)
ax.TickDir = 'out'; % Set ticks to point outwards

grid on;
