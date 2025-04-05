% --- This simulation is used to prove that the SE does not vary with
% warehouse width. The results of this simulation is not displayed on the
% report ---

clc; clear; close all;

% Warehouse Dimension Parameters
warehouse_width = (11:12:143);      % Number of columns (horizontal width of the warehouse grid in meters)
warehouse_depth = 19;               % Number of rows (vertical height of the warehouse grid in meters)

outdoor_width = 5;

rack_width = 3;                      % Horizontal width of each rack (columns it spans) (m)
rack_depth = 7;                      % Vertical height of each rack (rows it spans) (m)
row_spacing = 3;                     % Vertical spacing between rows of racks (in rows) (m)
col_spacing = 3;                     % Horizontal spacing between columns of racks (in columns) (m)

% Basic Signal Parameters
    Nmax = 100000;                   % Number of simulations
    S_t = 1;                         % Transmitted signal power
    S_int = 1;                       %
    a1 = 4.5;                        % Path Loss Exponent
    a2 = 3.5;
    a3 = 4;
    b = 3;
    W_out = 20;                      % External wall loss (dB)
    W_in = 10;                       % Internal wall loss (dB)
    R_BS = 500;                      % Inter BS distance (m)
    sigma_s_in = 6.8;                % Shadowing SD (dB)
    sigma_s_out = 4;
    m_nakagami = 1;                  % Nakagami parameter
    noise_power = -90;               % (dBm)
    N_0 = 10^(noise_power/10) / 1000;% Noise power (linear), sigma^2
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

% Step 1: Create the warehouse grid
for idx = 1:length(warehouse_width)

    current_width = warehouse_width(idx);
    total_width = current_width + outdoor_width;
    warehouse_grid = repmat('.', warehouse_depth, total_width);  % '.' represents an empty space
    
    % Step 2: Adding the racks to the warehouse with specified spacing
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

        middle_row = ceil(size(warehouse_grid, 1) / 2);  % Find the middle row
        middle_col = ceil(size(warehouse_grid, 2) / 2);  % Find the middle column

    end

    % Calculate the number of racks and IoT devices
    vertical_no_of_racks = ceil(warehouse_depth / (rack_depth + row_spacing));
    horizontal_no_of_racks = ceil(current_width / (rack_width + col_spacing));
    rack_count = vertical_no_of_racks * horizontal_no_of_racks;

    % Step 3: Place IoT Devices ("D") in the middle of each rack
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

                warehouse_grid(center_row, outdoor_width - 1) = 'L'; % L represents a Relay
                relay_coordinates = [relay_coordinates; center_row, outdoor_width - 1]; % Store the relay coordinates
            end
        end

    % Step 4: Place the Base Station (BS)
   
    warehouse_grid(middle_row, 1) = 'B';  % Mark the base station
    BS_coordinates = [middle_row, 1];

    % Step 5: Add a wall (W)

    warehouse_grid(:, outdoor_width) = 'W';

    % Step 6: Group devices and relays by row
    device_row_indices = ceil(device_coordinates(:, 1) / (rack_depth + row_spacing));
    relay_row_indices = ceil(relay_coordinates(:, 1) / (rack_depth + row_spacing));
    
    % Step 7: Assign each device to the relay in the same row
    assigned_relay = zeros(rack_count, 1);  % To store the relay index assigned to each device
    
    for i = 1:rack_count
        device_row = device_row_indices(i); % Get the row index for the device
        assigned_relay(i) = find(relay_row_indices == device_row, 1); % Find the corresponding relay for that row
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

    % Step 8: Monte-Carlo simulations
    for iter = 1 : Nmax
        % Find closest device
        user_index = randi(rack_count);
        relay_index = assigned_relay(user_index);
     
        user_distance_to_relay = device_to_relay_distances(user_index); % Device-to-Relay communication (D2R)      
        relay_distance_to_BS = relay_to_BS_distances(relay_index); % Relay-to-BS communication (R2B)
        
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

        P_int = sum(S_int * dist_int.^(-b) * 10^-(W_out/10) * X_s_out * X_m_3 );

        k = floor( user_distance_to_relay / ( rack_width + col_spacing ) );

        % SINR calculations
        SINR_3_67(idx, iter) = ( S_t * relay_distance_to_BS.^(-a1) * X_s_out * X_m_1 ) ./ ( P_int + N_0 );
        SINR_2(idx, iter) = ( S_t * relay_distance_to_BS.^(-a2) * X_s_out * X_m_1 ) ./ ( P_int + N_0 );
        SINR_3(idx, iter) = ( S_t * relay_distance_to_BS.^(-a3) * X_s_out * X_m_1 ) ./ ( P_int + N_0 );

        % SE calculations
        C_indoor_3_67(idx, iter) = log2( 1 + SINR_3_67(idx, iter) );
        C_indoor_2(idx, iter) = log2( 1 + SINR_2(idx, iter) );
        C_indoor_3(idx, iter) = log2( 1 + SINR_3(idx, iter) );

    end
    
    mean_SINR_3_67(idx) = mean(SINR_3_67(idx, :), 2);
    mean_SINR_2(idx) = mean(SINR_2(idx, :), 2);
    mean_SINR_3(idx) = mean(SINR_3(idx, :), 2);
    mean_C_3_67(idx) = mean(C_indoor_3_67(idx, :), 2);
    mean_C_2(idx) = mean(C_indoor_2(idx, :), 2);
    mean_C_3(idx) = mean(C_indoor_3(idx, :), 2);

end

mean_SINR_3_67_dB = 10*log10(mean_SINR_3_67);
mean_SINR_2_dB = 10*log10(mean_SINR_2);
mean_SINR_3_dB = 10*log10(mean_SINR_3);

figure;
plot(warehouse_width, mean_C_2, '-^', 'LineWidth', 2, 'Color', 'r');
hold on;
plot(warehouse_width, mean_C_3, '-x', 'LineWidth', 2, 'Color', 'g');
plot(warehouse_width, mean_C_3_67, '-o', 'LineWidth', 2, 'Color', 'b');
% Set Labels with Larger Font
xlabel('Warehouse Width w_w (m)', 'FontSize', 11);
ylabel('Mean SINR (dB)', 'FontSize', 11);

% Set Axis Font Size (This Increases Tick Labels!)
ax = gca; % Get current axes
ax.FontSize = 14; % Set larger font size for x and y axis values (tick labels)

grid on;
