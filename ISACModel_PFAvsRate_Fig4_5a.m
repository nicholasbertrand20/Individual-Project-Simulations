% --- This simulation is used to display Fig. 4.5a ---

clear; clc; close all;

% Parameters
L = 10;                                  % Time domain samples (size of Delay-Doppler operator matrix)
P_T = 20;                                % Power allocated to radar, equivalent to magnitude of s_r
Nmax = 1000000;                          % Number of simulations for Monte Carlo
sigma = 10^(-90/10) / 1000;              % Noise power (dBm)
gamma_C = 10^(0/10);                     % Gamma_C value (0 dB)

% Different values of gammaR in dB
DSNR_20 = 62.65;                         % D-SNR = 62.65 dB
DSNR_10 = 20;                            % D-SNR = 20 dB
DSNR_min10 = 10;                         % D-SNR = 10 dB

% Convert D-SNR from dB to linear scale
DSNR_20_linear = 10^(DSNR_20 / 10);
DSNR_10_linear = 10^(DSNR_10 / 10);
DSNR_min10_linear = 10^(DSNR_min10 / 10);

% X-axis values (threshold gamma)
Rate = linspace(0, 5, 30);
P_C = (2.^Rate - 1) / gamma_C;
gamma_val = 5;

threshold = sigma^2 * gamma_val;

% Calculate the absolute values for gamma_d based on D-SNR and noise power
abs_gamma_d_20 = sqrt(DSNR_20_linear * sigma^2);
abs_gamma_d_10 = sqrt(DSNR_10_linear * sigma^2);
abs_gamma_d_min10 = sqrt(DSNR_min10_linear * sigma^2);

% Storage for simulation results
PFA_results_20 = zeros(length(Rate), Nmax);
PFA_results_20_pergamma = zeros(length(Rate), 1);
PFA_results_10 = zeros(length(Rate), Nmax);
PFA_results_10_pergamma = zeros(length(Rate), 1);
PFA_results_min10 = zeros(length(Rate), Nmax);
PFA_results_min10_pergamma = zeros(length(Rate), 1);

% Monte Carlo simulations
for idx = 1:length(Rate)
    P_r = P_T - P_C(idx);         % Power allocated to radar
    s_r = sqrt(P_r / L) * ones(L, 1);

    if P_r < 0
        continue;   % Skip storing values if P_r is negative
    end

    for trial = 1:Nmax
        % Generate complex Gaussian noise nd_tilde and ns_tilde
        nd_tilde = (randn(L, 1) + 1j * randn(L, 1)) / sqrt(2) * sigma;
        ns_tilde = (randn(L, 1) + 1j * randn(L, 1)) / sqrt(2) * sigma;

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
        B_vector_20 = (xd_times_xdH_H0_20 + sigma^2 * gamma_val * eye(L)) / ...
                     (sum(abs(xd_tilde_H0_20).^2) + sigma^2 * gamma_val);
        B_vector_10 = (xd_times_xdH_H0_10 + sigma^2 * gamma_val * eye(L)) / ...
                     (sum(abs(xd_tilde_H0_10).^2) + sigma^2 * gamma_val);
        B_vector_min10 = (xd_times_xdH_H0_min10 + sigma^2 * gamma_val * eye(L)) / ...
                     (sum(abs(xd_tilde_H0_min10).^2) + sigma^2 * gamma_val);

        % Calculate test statistic
        xstildeH_B_xstilde_H0_20 = xs_tilde_H0_20' * B_vector_20 * xs_tilde_H0_20;
        xstildeH_B_xstilde_H0_10 = xs_tilde_H0_10' * B_vector_10 * xs_tilde_H0_10;
        xstildeH_B_xstilde_H0_min10 = xs_tilde_H0_min10' * B_vector_min10 * xs_tilde_H0_min10;

        % Count false alarm
        PFA_results_20(idx, trial) = double(abs(xstildeH_B_xstilde_H0_20) >= threshold);
        PFA_results_10(idx, trial) = double(abs(xstildeH_B_xstilde_H0_10) >= threshold);
        PFA_results_min10(idx, trial) = double(abs(xstildeH_B_xstilde_H0_min10) >= threshold);
    end

    % Calculate the probability of false alarm for the current gamma value
    PFA_results_20_pergamma(idx) = mean(PFA_results_20(idx,:), 2);
    PFA_results_10_pergamma(idx) = mean(PFA_results_10(idx,:), 2);
    PFA_results_min10_pergamma(idx) = mean(PFA_results_min10(idx,:), 2);

end

% Mask where P_r is negative
valid_indices = (P_C <= P_T);  % Only include points where P_C <= P_T

% Plotting results
figure;
semilogy(Rate(valid_indices), PFA_results_20_pergamma(valid_indices), '-o', 'LineWidth', 2, 'Color', 'blue');
hold on;
semilogy(Rate(valid_indices), PFA_results_10_pergamma(valid_indices), '-s', 'LineWidth', 2, 'Color', 'red');
semilogy(Rate(valid_indices), PFA_results_min10_pergamma(valid_indices), '-^', 'LineWidth', 2, 'Color', 'green');
xlabel('Spectral Efficiency (bps/Hz)', 'FontSize', 15);
ylabel('P_{FA} (Probability of False Alarm)', 'FontSize', 15);
ax = gca; % Get current axes
ax.FontSize = 12; % Set larger font size for x and y axis values (tick labels)
ax.TickDir = 'out'; % Set ticks to point outwards
legend('SINR = 62.65 dB (Calculated Warehouse SINR)', 'SINR = 20 dB', 'SINR = 10 dB', 'FontSize', 15)
grid on;