% --- This simulation is used to display Figure 4.1a ---

clc; clear; close all;

% --- Warehouse Dimension Parameters! ---

warehouse_width = (11:12:143);   % Number of columns (horizontal width of the warehouse grid in meters)
warehouse_depth = 19;            % Number of rows (vertical height of the warehouse grid in meters)

rack_width = 3;                  % Horizontal width of each rack (columns it spans) (m)
rack_depth = 7;                  % Vertical height of each rack (rows it spans) (m)
row_spacing = 3;                 % Vertical spacing between rows of racks (in rows) (m)
col_spacing = 3;                 % Horizontal spacing between columns of racks (in columns) (m)


% --- Signal Parameters! ---

Nmax = 100000;                   % Number of simulations
S_t = 1;                         % Transmitted signal power (W)
S_i = 1;                         % Interferer signal power (W)
a1 = 4.5;                        % Basic Path Loss Exponent for Signal
a2 = 3.5;
a3 = 4;
b = 3;                           % Basic Path Loss Exponent for Interferer
R_BS = 500;                      % BS Coverage radius
lambda_int = 2 / 100000;         % Interferer density (m^-2)
W_in = 10;
W_out = 20;
sigma_s_in = 6.8;                % Shadowing SD (dB)
sigma_s_out = 4;
m_nakagami = 1;                  % Nakagami parameter
noise_power = -90;               % (dBm)
N_0 = 10^(noise_power/10) / 1000;% Noise power (linear), sigma^2


% Initialisation of output - to create a matrix of required size
S_d_33 = zeros(length(warehouse_width), Nmax);
SINR_33 = zeros(length(warehouse_width), Nmax);
Rate_33 = zeros(length(warehouse_width), Nmax);

S_d_3 = zeros(length(warehouse_width), Nmax);
SINR_3 = zeros(length(warehouse_width), Nmax);
Rate_3 = zeros(length(warehouse_width), Nmax);

S_d_25 = zeros(length(warehouse_width), Nmax);
SINR_25 = zeros(length(warehouse_width), Nmax);
Rate_25 = zeros(length(warehouse_width), Nmax);

mean_rate_33 = zeros(length(warehouse_width), 1);
mean_rate_3 = zeros(length(warehouse_width), 1);
mean_rate_25 = zeros(length(warehouse_width), 1);

S_d_33_no_s = zeros(length(warehouse_width), Nmax);
SINR_33_no_s = zeros(length(warehouse_width), Nmax);
Rate_33_no_s = zeros(length(warehouse_width), Nmax);

S_d_3_no_s = zeros(length(warehouse_width), Nmax);
SINR_3_no_s = zeros(length(warehouse_width), Nmax);
Rate_3_no_s = zeros(length(warehouse_width), Nmax);

S_d_25_no_s = zeros(length(warehouse_width), Nmax);
SINR_25_no_s = zeros(length(warehouse_width), Nmax);
Rate_25_no_s = zeros(length(warehouse_width), Nmax);

mean_rate_33_no_s = zeros(length(warehouse_width), 1);
mean_rate_3_no_s = zeros(length(warehouse_width), 1);
mean_rate_25_no_s = zeros(length(warehouse_width), 1);

% --- Step 1: Create an empty warehouse grid ---
for idx = 1:length(warehouse_width)

    current_width = warehouse_width(idx);
    warehouse_grid = repmat('.', warehouse_depth, current_width);  % '.' represents an empty space
    
    % --- Step 2: Adding the racks to the warehouse with specified spacing ---

    for row_start = 2 : (rack_depth + row_spacing) : warehouse_depth
        for col_start = 2 : (rack_width + col_spacing) : current_width
            % Place a rack at (row_start, col_start) with specified width and height
            for i = 0 : rack_depth - 1
                for j = 0 : rack_width - 1
                    % Ensure row_start + i and col_start + j are within grid boundaries
                    if (row_start + i <= warehouse_depth) && (col_start + j <= current_width)
                        warehouse_grid(row_start + i, col_start + j) = 'R';  % Mark cell using 'R' as a Rack
                    end
                end
            end
        end
    end
        
        % --- Step 3: Place the Base Station (BS) ---

        middle_row = ceil(size(warehouse_grid, 1) / 2);  % Find the middle row
        middle_col = ceil(size(warehouse_grid, 2) / 2);  % Find the middle column
    
        warehouse_grid(middle_row, middle_col) = 'B';  % Mark the base station
        BS_coordinates = [middle_row, middle_col];
    
        % Calculate the number of racks and IoT devices
        vertical_no_of_racks = ceil( warehouse_depth / (rack_depth + row_spacing) );
        horizontal_no_of_racks = ceil( current_width / (rack_width + col_spacing) );
        rack_count = vertical_no_of_racks * horizontal_no_of_racks;
        iot_device_count = rack_count;  % Because each rack has one IoT device
    
        % --- Step 4: Place IoT Devices ("D") in the middle of each rack ---
        device_coordinates = zeros(iot_device_count, 2);  % List to store (x, y) coordinates of each device
    
        device_idx = 1;
            for row_start = 2 : (rack_depth + row_spacing) : warehouse_depth
                for col_start = 2 : (rack_width + col_spacing) : current_width
                    
                    % Calculate the center of the rack
                    device_row = row_start + floor(rack_depth / 2);
                    device_col = col_start + floor(rack_width / 2);
            
                    % Place the IoT device ("D") at the center
                    warehouse_grid(device_row, device_col) = 'D';
            
                    % Store the coordinates
                    device_coordinates(device_idx, :) = [device_row, device_col];
                    device_idx = device_idx + 1;
                end
            end
    
        % --- Step 5: Modelling Signal Path Loss ---
        
        BS_device_distances = zeros(size(device_coordinates, 1), 1); %Initialise the array to store BS-UE distance

        % Calculates the BS-UE distance
        for i = 1:iot_device_count
            BS_device_distances(i) = sqrt( (BS_coordinates(1) - device_coordinates(i, 1))^2 + ...
                                           (BS_coordinates(2) - device_coordinates(i, 2))^2 );
        end
        
        % --- Step 6: Monte-Carlo Simulation ---

        for iter = 1 : Nmax
            % Find closest device and their distance
            user_index = randi(iot_device_count);
            user_distance = sqrt((BS_coordinates(1) - device_coordinates(user_index, 1))^2 + ...
                                 (BS_coordinates(2) - device_coordinates(user_index, 2))^2);
    
            % Generating interference
            num_int = poissrnd(lambda_int * pi * R_BS^2);
            theta_int = 2 * pi * rand(num_int, 1);
            r_int = R_BS * sqrt(rand(num_int, 1));
    
            x_int = r_int .* cos(theta_int);
            y_int = r_int .* sin(theta_int);
    
            dist_int = sqrt(x_int.^2 + y_int.^2);
            
            k = floor( user_distance / ( rack_width + col_spacing ) );
    
            X_s_in = 10.^(normrnd(0, sigma_s_in)/10);
            X_s_out = 10.^(normrnd(0, sigma_s_out)/10);
    
            X_m_1 = gamrnd(m_nakagami, 1/m_nakagami);
            X_m_2 = gamrnd(m_nakagami, 1/m_nakagami);
 
            % Total interference power
            P_int = sum( S_t * ( dist_int ).^-b * 10^-(W_out/10) );
    
            % Desired signal powers
            S_d_33(idx, iter) = ( S_t * ( user_distance )^-a1 ...
                * 10^-(W_in*(k)/10) * X_s_in * X_m_1 );
    
            S_d_25(idx, iter) = ( S_t * ( user_distance )^-a2 ...
                * 10^-(W_in*(k)/10) * X_s_in * X_m_1 );
    
            S_d_3(idx, iter) = ( S_t * ( user_distance )^-a3 ...
                * 10^-(W_in*(k)/10) * X_s_in * X_m_1 );
    
            % SINR values
            SINR_33(idx, iter) = S_d_33(idx, iter) / ( P_int + N_0 ) ;
            SINR_25(idx, iter) = S_d_25(idx, iter) / ( P_int + N_0 ) ;
            SINR_3(idx, iter) = S_d_3(idx, iter) / ( P_int + N_0 ) ;
    
            % SE values
            Rate_33(idx, iter) = log2(1 + SINR_33(idx, iter));
            Rate_25(idx, iter) = log2(1 + SINR_25(idx, iter));
            Rate_3(idx, iter) = log2(1 + SINR_3(idx, iter));
    
            % Without Shadowing and Fading
            S_d_33_no_s(idx, iter) = ( S_t * ( user_distance )^-a1 ...
                * 10^-(W_in*(k)/10) );
    
            S_d_25_no_s(idx, iter) = ( S_t * ( user_distance )^-a2 ...
                * 10^-(W_in*(k)/10) );
    
            S_d_3_no_s(idx, iter) = ( S_t * ( user_distance )^-a3 ...
                * 10^-(W_in*(k)/10) );
    
            SINR_33_no_s(idx, iter) = S_d_33_no_s(idx, iter) / ( P_int + N_0 ) ;
            SINR_25_no_s(idx, iter) = S_d_25_no_s(idx, iter) / ( P_int + N_0 ) ;
            SINR_3_no_s(idx, iter) = S_d_3_no_s(idx, iter) / ( P_int + N_0 ) ;
    
            Rate_33_no_s(idx, iter) = log2(1 + SINR_33_no_s(idx, iter));
            Rate_25_no_s(idx, iter) = log2(1 + SINR_25_no_s(idx, iter));
            Rate_3_no_s(idx, iter) = log2(1 + SINR_3_no_s(idx, iter));
            
        end

    % Average all simulations
    mean_rate_33 = mean(Rate_33, 2);
    mean_rate_25 = mean(Rate_25, 2);
    mean_rate_3 = mean(Rate_3, 2);
    mean_rate_33_no_s = mean(Rate_33_no_s, 2);
    mean_rate_25_no_s = mean(Rate_25_no_s, 2);
    mean_rate_3_no_s = mean(Rate_3_no_s, 2);

end

% Plot ASE vs. Warehouse Area
figure;
plot(warehouse_width, mean_rate_25, '-^', 'LineWidth', 2, 'Color', 'r');
hold on;
plot(warehouse_width, mean_rate_3, '-x', 'LineWidth', 2, 'Color', 'g');
plot(warehouse_width, mean_rate_33, '-o', 'LineWidth', 2, 'Color', 'b');
plot(warehouse_width, mean_rate_25_no_s, '--', 'LineWidth', 2, 'Color', 'r');
plot(warehouse_width, mean_rate_3_no_s, '--', 'LineWidth', 2, 'Color', 'g');
plot(warehouse_width, mean_rate_33_no_s, '--', 'LineWidth', 2, 'Color', 'b');
xlabel('Warehouse Width w_w (m)', 'FontSize', 13);
ylabel('Mean SE (bps/Hz)', 'FontSize', 13);
ax = gca; % Get current axes
ax.FontSize = 12;
ax.TickDir = 'out';
legend("\alpha = 3.5", "\alpha = 4","\alpha = 4.5", "\alpha = 3.5 no shadowing & fading", "\alpha = 4 no shadowing & fading","\alpha = 4.5 no shadowing & fading", 'FontSize', 16);
grid on;