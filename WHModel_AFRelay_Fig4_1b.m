% --- This simulation is used to display Fig. 4.1b

clc; clear; close all;

% --- Warehouse Dimension Parameters ---

warehouse_width = (11:12:143);   % Number of columns (horizontal width of the warehouse grid in meters)
warehouse_depth = 19;            % Number of rows (vertical height of the warehouse grid in meters)

outdoor_width = 5;               % Relay-BS distance (m)
rack_width = 3;                  % Horizontal width of each rack (columns it spans) (m)
rack_depth = 7;                  % Vertical height of each rack (rows it spans) (m)
row_spacing = 3;                 % Vertical spacing between rows of racks (in rows) (m)
col_spacing = 3;                 % Horizontal spacing between columns of racks (in columns) (m)

% --- Signal Parameters ---
Nmax = 100000;                   % Number of simulations
S_t = 1;                         % Transmitted signal power (W)
S_int = 1;                       % Interferer signal power (W)
G = 10^(40/10);                  % Gain of relay (40 dB)
a1 = 4.5;                        % Basic Path Loss Exponent for Desired signal
a2 = 3.5;
a3 = 4;
b = 3;                           % Basic Path Loss Exponent for Interferer
W_out = 20;                      % External wall loss (dB)
W_in = 10;                       % Internal wall loss (dB)
R_BS = 500;                      % Inter BS distance (m)
sigma_s_in = 6.8;                % Shadowing SD (dB)
sigma_s_out = 4;
m_nakagami = 1;                  % Nakagami parameter
noise_power = -90;               % (dBm)
N_0 = 10^(noise_power/10) / 1000;% Noise power (linear), sigma^2

% PPP Parameters for interferers
lambda_int = 2 / 100000;         % Density of interferers per m2

% Initialisation for future key performance metrics
SINR_3_67 = zeros(length(warehouse_width), Nmax);
SINR_3 = zeros(length(warehouse_width), Nmax);
SINR_2 = zeros(length(warehouse_width), Nmax);

mean_SINR_3_67 = zeros(length(warehouse_width), 1);
mean_SINR_3 = zeros(length(warehouse_width), 1);
mean_SINR_2 = zeros(length(warehouse_width), 1);

C_indoor_3_67 = zeros(length(warehouse_width), Nmax);
mean_C_3_67 = zeros(length(warehouse_width), 1);
C_indoor_2 = zeros(length(warehouse_width), Nmax);
mean_C_2 = zeros(length(warehouse_width), 1);
C_indoor_3 = zeros(length(warehouse_width), Nmax);
mean_C_3 = zeros(length(warehouse_width), 1);
    
% --- Step 1: Create the warehouse grid ---
for idx = 1:length(warehouse_width)

    current_width = warehouse_width(idx);
    total_width = current_width + outdoor_width;
    warehouse_grid = repmat('.', warehouse_depth, total_width);  % '.' represents an empty space
    
    % --- Step 2: Adding the racks to the warehouse with specified spacing ---

    for row_start = 2 : (rack_depth + row_spacing) : warehouse_depth
        for col_start = 2 + outdoor_width : (rack_width + col_spacing) : total_width
            % Place a rack at (row_start, col_start) with specified width and height
            for i = 0 : rack_depth - 1
                for j = 0 : rack_width - 1
                    % Ensure row_start + i and col_start + j are within grid boundaries
                    if (row_start + i <= warehouse_depth) && (col_start + j <= total_width)
                        warehouse_grid(row_start + i, col_start + j) = 'R';  % Mark cell as a Rack
                    end
                end
            end
        end
    end

    % Calculate the number of racks and IoT devices

    middle_row = ceil(size(warehouse_grid, 1) / 2);  % Find the middle row
    middle_col = ceil(size(warehouse_grid, 2) / 2);  % Find the middle column

    vertical_no_of_racks = ceil(warehouse_depth / (rack_depth + row_spacing));
    horizontal_no_of_racks = ceil(current_width / (rack_width + col_spacing));
    rack_count = vertical_no_of_racks * horizontal_no_of_racks;

    % --- Step 3: Place IoT Devices ("D") in the middle of each rack and Relays ("L") ---

    device_coordinates = zeros(rack_count, 2);  % List to store (x, y) coordinates of each device
    relay_coordinates = [];
    device_idx = 1;
        for row_start = 2 : (rack_depth + row_spacing) : warehouse_depth 
            for col_start = 2 + outdoor_width : (rack_width + col_spacing) : total_width
                % Calculate the height and width of the current rack
                current_rack_depth = min(rack_depth, warehouse_depth - row_start + 1);
                current_rack_width = min(rack_width, total_width - col_start + 1);
        
                % Calculate the center of the rack
                center_row = row_start + floor(current_rack_depth / 2);
                center_col = col_start + floor(current_rack_width / 2);
        
                % Place the IoT device ("D") at the center
                warehouse_grid(center_row, center_col) = 'D';
        
                % Store the coordinates
                device_coordinates(device_idx, :) = [center_row, center_col];
                device_idx = device_idx + 1;

                warehouse_grid(center_row, outdoor_width - 1) = 'L'; % L represents the Relay
                relay_coordinates = [relay_coordinates; center_row, outdoor_width - 1]; % Store the coordinates of the relay connected to the respective UE
            end
        end

    % --- Step 4: Place the Base Station (BS) ---
   
    warehouse_grid(middle_row, 1) = 'B';  % Mark the base station
    BS_coordinates = [middle_row, 1];

    % --- Step 5: Add the wall (W) ---

    warehouse_grid(:, outdoor_width) = 'W';

    % Group devices and relays by row
    device_row_indices = ceil(device_coordinates(:, 1) / (rack_depth + row_spacing));       % This matrix stores the nth row of racks
    relay_row_indices = ceil(relay_coordinates(:, 1) / (rack_depth + row_spacing));         % This matrix stores the relay connected to each device, 1 is the top relay and 2 is the bottom relay
    
    % --- Step 6: Assign each device to the relay in the same row and calculate the distances ---

    assigned_relay = zeros(rack_count, 1);  % To store the relay index assigned to each device
    
    for i = 1:rack_count
        device_row = device_row_indices(i);  % Get the row index for the device
        assigned_relay(i) = find(relay_row_indices == device_row, 1);  % Find the corresponding relay for that row
    end

    device_to_relay_distances = zeros(rack_count, 1);  % Distance from device to assigned relay

    for i = 1:rack_count
        relay_idx = assigned_relay(i);
        device_to_relay_distances(i) = sqrt((relay_coordinates(relay_idx, 1) - device_coordinates(i, 1))^2 + ...
                                            (relay_coordinates(relay_idx, 2) - device_coordinates(i, 2))^2);
    end
    
    relay_to_BS_distances = zeros(size(relay_coordinates, 1), 1);  % Distance from relay to BS
    
    for j = 1:size(relay_coordinates, 1)
        relay_to_BS_distances(j) = sqrt((BS_coordinates(1) - relay_coordinates(j, 1))^2 + ...
                                        (BS_coordinates(2) - relay_coordinates(j, 2))^2);
    end

    % --- Step 7: Monte-Carlo Simulation ---

    for iter = 1 : Nmax
        % Find closest device
        user_index = randi(rack_count);
        relay_index = assigned_relay(user_index);
    
        user_distance_to_relay = device_to_relay_distances(user_index);
        relay_distance_to_BS = relay_to_BS_distances(relay_index);
        
        % Generate interference
        num_int = poissrnd(lambda_int * pi * R_BS^2);
        theta_int = 2 * pi * rand(num_int, 1);
        r_int = R_BS * sqrt(rand(num_int, 1));

        x_int = r_int .* cos(theta_int);
        y_int = r_int .* sin(theta_int);

        dist_int = sqrt(x_int.^2 + y_int.^2);

        % Shadowing parameters
        X_s_in = 10.^(normrnd(0, sigma_s_in)/10);
        X_s_out = 10.^(normrnd(0, sigma_s_out)/10);

        % Fading parameters
        X_m_1 = gamrnd(m_nakagami, 1/m_nakagami);
        X_m_2 = gamrnd(m_nakagami, 1/m_nakagami);
        X_m_3 = gamrnd(m_nakagami, 1/m_nakagami);

        % Total interference power
        P_int = sum(S_int * dist_int.^(-b) * 10^-(W_out/10) * X_s_out * X_m_3 );

        k = floor( user_distance_to_relay / ( rack_width + col_spacing ) );

        % Desired signal power
        S_r_3_67 = G * ( S_t * user_distance_to_relay.^(-a1) * 10^-(W_out/10) * 10^-(W_in*k/10) );
        S_r_2 = G * ( S_t * user_distance_to_relay.^(-a2) * 10^-(W_out/10) * 10^-(W_in*k/10) );
        S_r_3 = G * ( S_t * user_distance_to_relay.^(-a3) * 10^-(W_out/10) * 10^-(W_in*k/10) );

        % SINR calculations
        SINR_3_67(idx, iter) = ( S_r_3_67 * relay_distance_to_BS.^(-a1) * X_s_out * X_m_1 ) ./ ( ( S_t * ( relay_distance_to_BS + ...
            user_distance_to_relay ).^(-a1) * 10^-(W_out/10) * 10^-(W_in*k/10) * X_s_in * X_m_2 ) + P_int + N_0 * ( 1 + G) );

        SINR_2(idx, iter) = ( S_r_2 * relay_distance_to_BS.^(-a2) * X_s_out * X_m_1 ) ./ ( ( S_t * ( relay_distance_to_BS + ...
            user_distance_to_relay ).^(-a2) * 10^-(W_out/10) * 10^-(W_in*k/10) * X_s_in * X_m_2 ) + P_int + N_0 * ( 1 + G) );

        SINR_3(idx, iter) = ( S_r_3 * relay_distance_to_BS.^(-a3) * X_s_out * X_m_1 ) ./ ( ( S_t * ( relay_distance_to_BS + ...
            user_distance_to_relay ).^(-a3) * 10^-(W_out/10) * 10^-(W_in*k/10) * X_s_in * X_m_2 ) + P_int + N_0 * ( 1 + G) );

        % SE calculations
        C_indoor_3_67(idx, iter) = log2( 1 + SINR_3_67(idx, iter) );

        C_indoor_2(idx, iter) = log2( 1 + SINR_2(idx, iter) );

        C_indoor_3(idx, iter) = log2( 1 + SINR_3(idx, iter) );

    end
    
    % Mean the simulations
    mean_SINR_3_67(idx) = mean(SINR_3_67(idx, :), 2);
    mean_SINR_2(idx) = mean(SINR_2(idx, :), 2);
    mean_SINR_3(idx) = mean(SINR_3(idx, :), 2);
    mean_C_3_67(idx) = mean(C_indoor_3_67(idx, :), 2);
    mean_C_2(idx) = mean(C_indoor_2(idx, :), 2);
    mean_C_3(idx) = mean(C_indoor_3(idx, :), 2);

end

figure;
plot(warehouse_width, mean_C_2, '-^', 'LineWidth', 2, 'Color', 'r');
hold on;
plot(warehouse_width, mean_C_3, '-x', 'LineWidth', 2, 'Color', 'g');
plot(warehouse_width, mean_C_3_67, '-o', 'LineWidth', 2, 'Color', 'b');
% Set Labels with Larger Font
xlabel('Warehouse Width w_w (m)', 'FontSize', 13);
ylabel('Mean SE (bps/Hz)', 'FontSize', 13);

% Set Axis Font Size (This Increases Tick Labels!)
ax.TickDir = 'out'; % Set ticks to point outwards
ax.FontSize = 16; % Increase the font size of tick labels

legend("\alpha = 3.5", "\alpha = 4","\alpha = 4.5", 'FontSize', 20);
grid on;