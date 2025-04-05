% --- This simulation is  used to display Fig. 4.6b and 4.6d ---

clear; clc; close all;
 
% Parameters
L = 10;                                % Time domain samples (size of Delay-Doppler operator matrix)
P_r = 5;                               % Total power (W)

    gamma_T = 10;                          % dB, switch between 0 and 10 to switch between Fig. 4.6b and 4.6d

Nmax = 100000;                         % Number of simulations for Monte Carlo
sigma = 10^(-90/10) / 1000;
s_r = sqrt(P_r / L) * ones(L, 1);
gamma_val = linspace(0, 10, 20);

% Different values of D-SNR in dB
gammaR_20 = 62.65;                     % D-SNR = 62.65 dB
gammaR_10 = 20;                        % D-SNR = 20 dB
gammaR_min10 = 10;                     % D-SNR = 10 dB

% Convert D-SNR from dB to linear scale
gammaR_20_linear = 10^(gammaR_20 / 10);
gammaR_10_linear = 10^(gammaR_10 / 10);
gammaR_min10_linear = 10^(gammaR_min10 / 10);

% Convert gamma_T and gamma_C to linear
gamma_T_linear = 10^(gamma_T / 10);

% Calculate the absolute values for gamma_d based on D-SNR and noise power
abs_gammad_20 = sqrt(gammaR_20_linear * sigma^2 / P_r);
abs_gammad_10 = sqrt(gammaR_10_linear * sigma^2 / P_r);
abs_gammad_min10 = sqrt(gammaR_min10_linear * sigma^2 / P_r);
abs_gamma_t = sqrt(gamma_T_linear * sigma^2 / P_r);

% Storage for simulation results
PD_results_20 = zeros(Nmax, 1);
PD_results_20_perrate = zeros(length(gamma_val), 1);
PD_results_10 = zeros(Nmax, 1);
PD_results_10_perrate = zeros(length(gamma_val), 1);
PD_results_min10 = zeros(Nmax, 1);
PD_results_min10_perrate = zeros(length(gamma_val), 1);
PD_theo_results = zeros(length(gamma_val), 1);

% Monte-Carlo simulation
for idx = 1:length(gamma_val)
    gam = gamma_val(idx);

    for trial = 1:Nmax
        nd_tilde = (randn(L, 1) + 1j * randn(L, 1)) / sqrt(2) * sigma;
        ns_tilde = (randn(L, 1) + 1j * randn(L, 1)) / sqrt(2) * sigma;

        % Calculating xd tilde and xs tilde using the formula from research paper
        xd_tilde_H1_20 = abs_gammad_20 * s_r + nd_tilde;
        xs_tilde_H1_20 = abs_gamma_t * s_r + ns_tilde;
        xd_tilde_H1_10 = abs_gammad_10 * s_r + nd_tilde;
        xs_tilde_H1_10 = abs_gamma_t * s_r + ns_tilde;
        xd_tilde_H1_min10 = abs_gammad_min10 * s_r + nd_tilde;
        xs_tilde_H1_min10 = abs_gamma_t * s_r + ns_tilde;

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
    PD_results_20_perrate(idx) = mean(PD_results_20);
    PD_results_10_perrate(idx) = mean(PD_results_10);
    PD_results_min10_perrate(idx) = mean(PD_results_min10);
end

% Plotting the results
figure;
hold on;
plot(gamma_val, PD_results_min10_perrate, 'g^-', 'LineWidth', 2);
plot(gamma_val, PD_results_10_perrate, 'rs-', 'LineWidth', 2);
plot(gamma_val, PD_results_20_perrate, 'bo-', 'LineWidth', 2);
xlabel('\gamma_{thr} (SINR Threshold)', 'FontSize', 15);
ylabel('P_D (Probability of Detection)', 'FontSize', 15);
ax = gca;  
ax.FontSize = 12;  
ax.TickDir = 'out';  
ax.LineWidth = 1.5;  
ax.Box = 'on';  
ax.Layer = 'top';  
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 0.5);
legend('SINR = 10 dB', 'SINR = 20 dB', 'SINR = 62.65 dB', 'FontSize', 15);
grid on;