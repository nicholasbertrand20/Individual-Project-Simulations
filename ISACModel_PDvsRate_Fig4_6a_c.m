% --- This simulation is used to display Fig. 4.6a and Fig. 4.6c ---

clear; clc; close all;

% Parameters
L = 10;                                % Time domain samples (size of Delay-Doppler operator matrix)
P_T = 20;                              % Total power (W)
gamma_T = 0;                           % dB - scalar coefficient channel of radar
gamma_C = 0;                           % dB - ratio of the squared absolute value of the transmitter–CR channel to the variance of additive noise at the CR.
Nmax = 100000;                         % Number of simulations for Monte Carlo
sigma = 10^(-90/10) / 1000;

% Different values of D-SNR in dB
gammaR_20 = 62.65;                     % D-SNR = 62.65 dB
gammaR_10 = 20;                        % D-SNR = 20 dB
gammaR_min10 = 10;                     % D-SNR = 10 dB
gam = 5;                               % SINR Threshold (dB)

% Convert D-SNR from dB to linear scale
gammaR_20_linear = 10^(gammaR_20 / 10);
gammaR_10_linear = 10^(gammaR_10 / 10);
gammaR_min10_linear = 10^(gammaR_min10 / 10);

% Convert gamma_T and gamma_C to linear
gamma_T_linear = 10^(gamma_T / 10);
gamma_C_linear = 10^(gamma_C / 10);

% Creating x-axis with rate and calculate P_C for each rate value
Rate_range = linspace(0, 5, 50);
P_C = (2.^Rate_range - 1) / gamma_C_linear; 

% Calculate the absolute values for gamma_d based on D-SNR and noise power
% - Scalar channel coefficient of communication system
abs_gammad_20 = sqrt(gammaR_20_linear * sigma^2);
abs_gammad_10 = sqrt(gammaR_10_linear * sigma^2);
abs_gammad_min10 = sqrt(gammaR_min10_linear * sigma^2);
abs_gamma_t = sqrt(gamma_T_linear * sigma^2);

% Storage for simulation results
PD_results_20 = zeros(Nmax, 1);
PD_results_20_perrate = zeros(length(Rate_range), 1);
PD_results_10 = zeros(Nmax, 1);
PD_results_10_perrate = zeros(length(Rate_range), 1);
PD_results_min10 = zeros(Nmax, 1);
PD_results_min10_perrate = zeros(length(Rate_range), 1);
PD_theo_results = zeros(length(Rate_range), 1);

% Start Monte-Carlo simulation
for rate_idx = 1:length(Rate_range)
    P_r = P_T - P_C(rate_idx);         % Power allocated to radar
    s_r = sqrt(P_r / L) * ones(L, 1);

    if P_r < 0
        continue;  % Skip storing values if P_r is negative
    end

    for trial = 1:Nmax
        nd_tilde = (randn(L, 1) + 1j * randn(L, 1)) / sqrt(2) * sigma;
        ns_tilde = (randn(L, 1) + 1j * randn(L, 1)) / sqrt(2) * sigma;

        % Setting random phase for gamma to get RV of non-absolute gam_d and gam_t
        gamma_d_phase = 2 * pi * rand();
        gamma_t_phase = 2 * pi * rand();
        gamma_d_val_20 = abs_gammad_20 * (cos(gamma_d_phase) + 1j * sin(gamma_d_phase));
        gamma_t_val_20 = abs_gamma_t * (cos(gamma_t_phase) + 1j * sin(gamma_t_phase));
        gamma_d_val_10 = abs_gammad_10 * (cos(gamma_d_phase) + 1j * sin(gamma_d_phase));
        gamma_t_val_10 = abs_gamma_t * (cos(gamma_t_phase) + 1j * sin(gamma_t_phase));
        gamma_d_val_min10 = abs_gammad_min10 * (cos(gamma_d_phase) + 1j * sin(gamma_d_phase));
        gamma_t_val_min10 = abs_gamma_t * (cos(gamma_t_phase) + 1j * sin(gamma_t_phase));

        % Calculating xd tilde and xs tilde using the formula from research paper
        xd_tilde_H1_20 = gamma_d_val_20 * s_r + nd_tilde;
        xs_tilde_H1_20 = gamma_t_val_20 * s_r + ns_tilde;
        xd_tilde_H1_10 = gamma_d_val_10 * s_r + nd_tilde;
        xs_tilde_H1_10 = gamma_t_val_10 * s_r + ns_tilde;
        xd_tilde_H1_min10 = gamma_d_val_min10 * s_r + nd_tilde;
        xs_tilde_H1_min10 = gamma_t_val_min10 * s_r + ns_tilde;

        % Calculate B vector
        B_vector_20 = (xd_tilde_H1_20 * xd_tilde_H1_20' + sigma^2 * gam * eye(L)) / ...
                      (sum(abs(xd_tilde_H1_20).^2) + sigma^2 * gam);
        B_vector_10 = (xd_tilde_H1_10 * xd_tilde_H1_10' + sigma^2 * gam * eye(L)) / ...
                      (sum(abs(xd_tilde_H1_10).^2) + sigma^2 * gam);
        B_vector_min10 = (xd_tilde_H1_min10 * xd_tilde_H1_min10' + sigma^2 * gam * eye(L)) / ...
                      (sum(abs(xd_tilde_H1_min10).^2) + sigma^2 * gam);

        % Calculate PD
        xstildeH_B_xstilde_H1_20 = xs_tilde_H1_20' * B_vector_20 * xs_tilde_H1_20;
        xstildeH_B_xstilde_H1_10 = xs_tilde_H1_10' * B_vector_10 * xs_tilde_H1_10;
        xstildeH_B_xstilde_H1_min10 = xs_tilde_H1_min10' * B_vector_min10 * xs_tilde_H1_min10;

        threshold = sigma^2 * gam;

        % Count false alarm
        PD_results_20(trial) = double(abs(xstildeH_B_xstilde_H1_20) >= threshold);
        PD_results_10(trial) = double(abs(xstildeH_B_xstilde_H1_10) >= threshold);
        PD_results_min10(trial) = double(abs(xstildeH_B_xstilde_H1_min10) >= threshold);
    end

    % Calculate the probability of detection for the current gamma value
    PD_results_20_perrate(rate_idx) = mean(PD_results_20);
    PD_results_10_perrate(rate_idx) = mean(PD_results_10);
    PD_results_min10_perrate(rate_idx) = mean(PD_results_min10);
end

% Mask where P_r is negative
valid_indices = (P_C <= P_T);  % Only include points where P_C <= P_T

% Plotting the results
figure;
hold on;
plot(Rate_range(valid_indices), PD_results_min10_perrate(valid_indices), 'g^-', 'LineWidth', 2);
plot(Rate_range(valid_indices), PD_results_10_perrate(valid_indices), 'rs-', 'LineWidth', 2);
plot(Rate_range(valid_indices), PD_results_20_perrate(valid_indices), 'bo-', 'LineWidth', 2);
xlabel('Spectral Efficiency (bps/Hz)', 'FontSize', 15);
ylabel('P_D (Probability of Detection)', 'FontSize', 15);
ax = gca;  
ax.FontSize = 12;  
ax.TickDir = 'out';  
ax.LineWidth = 1.5;  
ax.Box = 'on';  
ax.Layer = 'top';  
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 0.5);
set(ax, 'YScale', 'log'); 
legend('SINR = 10 dB', 'SINR = 20 dB', 'SINR = 62.65 dB', 'FontSize', 15);
grid on;