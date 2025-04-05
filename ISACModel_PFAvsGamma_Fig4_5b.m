% --- This simulation is used to produce Fig. 4.5b ---

clc; clear; close all;

% Basic Signal Parameters
Nmax = 100000;                         % Number of simulations

% Parameters
L = 10;                              % Time domain samples (size of Delay-Doppler operator matrix)
P_r = 5;                             % Power allocated to radar, equivalent to magnitude of s_r
s_r = sqrt(P_r / L) * ones(L, 1);
sigma = (10^(-90/10) / 1000);

% Different values of D-SNR in dB
DSNR_20 = 62.65;                    % D-SNR = 62.65 dB
DSNR_10 = 20;                       % D-SNR = 20 dB
DSNR_min10 = 10;                    % D-SNR = 10 dB
    
% Convert D-SNR from dB to linear scale
DSNR_20_linear = 10^(DSNR_20 / 10);
DSNR_10_linear = 10^(DSNR_10 / 10);
DSNR_min10_linear = 10^(DSNR_min10 / 10);
    
% Calculate the absolute values for gamma_d based on D-SNR and noise power
abs_gamma_d_20 = sqrt(DSNR_20_linear * sigma^2 / P_r);
abs_gamma_d_10 = sqrt(DSNR_10_linear * sigma^2 / P_r);
abs_gamma_d_min10 = sqrt(DSNR_min10_linear * sigma^2 / P_r);

% X-axis values (threshold gamma)
gamma_val = linspace(0, 10, 20);
    
% Initialisation for future key performance metrics
SINR_4_5 = zeros(1, Nmax);
SINR_4 = zeros(1, Nmax);
SINR_3_5 = zeros(1, Nmax);

% Storage for simulation results
PFA_results_20 = zeros(Nmax, 1);
PFA_results_20_pergamma = zeros(length(gamma_val), 1);
PFA_results_10 = zeros(Nmax, 1);
PFA_results_10_pergamma = zeros(length(gamma_val), 1);
PFA_results_min10 = zeros(Nmax, 1);
PFA_results_min10_pergamma = zeros(length(gamma_val), 1);

% Monte Carlo simulations
for gamma_idx = 1:length(gamma_val)
    gam = gamma_val(gamma_idx);

    for trial = 1:Nmax
        % Generate complex Gaussian noise nd_tilde and ns_tilde
        nd_tilde = (randn(L, 1) + 1j * randn(L, 1)) * sqrt(sigma^2/2);
        ns_tilde = (randn(L, 1) + 1j * randn(L, 1)) * sqrt(sigma^2/2);

        % Compute H0 components
        xd_tilde_H0_20 = abs_gamma_d_20 * s_r + nd_tilde;
        xs_tilde_H0_20 = ns_tilde;
        xd_tilde_H0_10 = abs_gamma_d_10 * s_r + nd_tilde;
        xs_tilde_H0_10 = ns_tilde;
        xd_tilde_H0_min10 = abs_gamma_d_min10 * s_r + nd_tilde;
        xs_tilde_H0_min10 = ns_tilde;

        xd_times_xdH_H0_20 = xd_tilde_H0_20 * xd_tilde_H0_20';
        xd_times_xdH_H0_10 = xd_tilde_H0_10 * xd_tilde_H0_10';
        xd_times_xdH_H0_min10 = xd_tilde_H0_min10 * xd_tilde_H0_min10';  

        % Calculate B vector
        B_vector_20 = (xd_times_xdH_H0_20 + sigma^2 * gam * eye(L)) / ...
                     (sum(abs(xd_tilde_H0_20).^2) + sigma^2 * gam);
        B_vector_10 = (xd_times_xdH_H0_10 + sigma^2 * gam * eye(L)) / ...
                     (sum(abs(xd_tilde_H0_10).^2) + sigma^2 * gam);
        B_vector_min10 = (xd_times_xdH_H0_min10 + sigma^2 * gam * eye(L)) / ...
                     (sum(abs(xd_tilde_H0_min10).^2) + sigma^2 * gam);

        % Calculate test statistic
        xstildeH_B_xstilde_H0_20 = xs_tilde_H0_20' * B_vector_20 * xs_tilde_H0_20;
        xstildeH_B_xstilde_H0_10 = xs_tilde_H0_10' * B_vector_10 * xs_tilde_H0_10;
        xstildeH_B_xstilde_H0_min10 = xs_tilde_H0_min10' * B_vector_min10 * xs_tilde_H0_min10;

        threshold = sigma^2 * gam;

        % Count false alarm probability
        PFA_results_20(trial) = double(abs(xstildeH_B_xstilde_H0_20) >= threshold);
        PFA_results_10(trial) = double(abs(xstildeH_B_xstilde_H0_10) >= threshold);
        PFA_results_min10(trial) = double(abs(xstildeH_B_xstilde_H0_min10) >= threshold);
    end

    % Calculate the probability of false alarm for the current gamma value
    PFA_results_20_pergamma(gamma_idx) = mean(PFA_results_20);
    PFA_results_10_pergamma(gamma_idx) = mean(PFA_results_10);
    PFA_results_min10_pergamma(gamma_idx) = mean(PFA_results_min10);
end

figure;
hold on;
plot(gamma_val, PFA_results_20_pergamma, '-o', 'LineWidth', 2, 'Color', 'blue');
plot(gamma_val, PFA_results_10_pergamma, '-s', 'LineWidth', 2, 'Color', 'red');
plot(gamma_val, PFA_results_min10_pergamma, '-^', 'LineWidth', 2, 'Color', 'green');
xlabel('\gamma_{thr} (SINR Threshold)', 'FontSize', 15);
ylabel('P_{FA} (Probability of False Alarm)', 'FontSize', 15);
ax = gca;  
ax.FontSize = 12;  
ax.TickDir = 'out';  
ax.LineWidth = 1.5;  
ax.Box = 'on';  
ax.Layer = 'top';  
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 0.5);
legend('SINR = 62.65 dB (Calculated Warehouse SINR)', 'SINR = 20 dB', 'SINR = 10 dB', 'FontSize', 15)
grid on;

