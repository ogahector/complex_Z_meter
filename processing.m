close all; clc; clear
save_plots = true;  % Set this to true to save the plots, or false to not save

% List of folders to search
folder_list = { 'measurement_data/D20250325_E01_U_1k_1k', ...
                'measurement_data/D20250325_E01_U_1M_1k',...
                'measurement_data/D20250325_E01_U_1mH_1k',...
                'measurement_data/D20250325_E01_U_1nF_1k',...
                'measurement_data/D20250325_E01_U_10mH_1k',...
                'measurement_data/D20250325_E01_U_10nF_1k',...
                'measurement_data/D20250325_E01_U_10uF_1k',...
                'measurement_data/D20250325_E01_U_11k_1k',...
                'measurement_data/D20250325_E01_U_90k_1k',...
                'measurement_data/D20250325_E01_U_100_1k',...
                'measurement_data/D20250325_E01_U_100uF_1k',...
                'measurement_data/D20250325_E01_U_300k_1k',...
                'measurement_data/D20250325_E01_U_3260_1k',...
                'measurement_data/D20250325_E01_U_4700nF_1k',...
                'measurement_data/D20250325_E01_U_4700uH_1k',...
                'measurement_data/D20250325_E01_U_32800_1k', ...
                'measurement_data/D20250326_E02_U_R0_L1mH_C1nF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L1mH_C1uF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L1mH_C10nF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L10mH_C1uF_PAR_1k', ...
                'measurement_data/D20250326_E02_U_R0_L10mH_C1uF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L100mH_C1nF_PAR_1k', ...
                'measurement_data/D20250326_E02_U_R0_L100mH_C10nF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L100mH_C100nF_PAR_1k',...
                'measurement_data/D20250326_E02_U_R0_L100mH_C100nF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L4700uH_C1uF_PAR_1k', ...
                'measurement_data/D20250326_E02_U_R0_L4700uH_C1uF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L4700uH_C100nF_SER_1k'};

% Series model (RLC all in series)
series_model = @(params, freq) (params(1) + 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3)));
% Resistor model (RL in series, with C in shunt)
resistor_model = @(params, freq) 1 ./ (1i .* 2 .* pi .* freq .* params(3) + 1 ./ (params(1) + 1i .* 2 .* pi .* freq .* params(2)));
% High value resistor model
high_value_res_model = @(params, freq) 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3) + 1 ./ params(1));
% Parallel model (RLC all in parallel)
parallel_model = @(params, freq) 1 ./ (params(1) + 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3)));

%% Initialize cell array to store Z values
Z_values = cell(length(folder_list), 1);

% Loop through each folder in the folder_list
for i = 1:length(folder_list)
    % Extract the folder name
    folder_name = folder_list{i};
    
    % Use regular expression to extract the value between 'U_' and '_1k'
    expr = 'U_(.*?)_1k';  % Regex pattern to extract the value between U_ and _1k
    match = regexp(folder_name, expr, 'tokens');
    
    if ~isempty(match)
        % Extract the matched value
        value = match{1}{1};
        
        % Initialize Z as an empty cell array
        Z = {};
        
        % Check if the value contains 'F' for capacitance or 'H' for inductance
        if contains(value, 'F')
            % Convert to scientific notation for capacitance
            num = str2double(regexprep(value, '[^\d.]', ''));  % Remove non-numeric characters (e.g., 'p', 'n', etc.)
            if contains(value, 'p')
                num = num * 1e-12;  % picofarads (pF)
            elseif contains(value, 'n')
                num = num * 1e-9;   % nanofarads (nF)
            elseif contains(value, 'u')
                num = num * 1e-6;   % microfarads (uF)
            elseif contains(value, 'm')
                num = num * 1e-3;   % millifarads (mF)
            elseif contains(value, 'k')
                num = num * 1e3;    % kilofarads (kF)
            end
            Z = {'C', num};  % Store the type 'C' and the numerical value in scientific notation
        elseif contains(value, 'H')
            % Convert to inductance (in Henrys)
            num = str2double(regexprep(value, '[^\d.]', ''));  % Remove non-numeric characters (e.g., 'm', 'u', etc.)
            if contains(value, 'm')
                num = num * 1e-3;  % millihenrys (mH)
            elseif contains(value, 'u')
                num = num * 1e-6;  % microhenrys (uH)
            elseif contains(value, 'k')
                num = num * 1e3;   % kilohenrys (kH)
            end
            Z = {'L', num};  % Store the type 'L' and the numerical value in Henrys
        else
            % For resistance, remove non-numeric characters and convert to ohms
            num = str2double(regexprep(value, '[^\d.]', ''));
            if contains(value, 'k')
                num = num * 1000;  % kilo (k)
            elseif contains(value, 'M')
                num = num * 1e6;   % mega (M)
            elseif contains(value, 'm')
                num = num * 1e-3;  % milli (m)
            elseif contains(value, 'u')
                num = num * 1e-6;  % micro (u)
            end
            Z = {'R', num};  % Store the type 'R' and the numerical value in ohms
        end
        
        % Store the Z value for the current folder
        Z_values{i} = Z;
    else
        warning('No matching value found in folder: %s', folder_name);
    end
end

%% Pre-allocate a cell array to store the results (3 arrays per file)
num_files = length(folder_list);
data_cell = cell(3, num_files);  % 3 rows: frequency, magnitude, phase for each file

% Loop through each folder
for i = 1:num_files
    % Get the current folder path
    folder_path = folder_list{i};
    
    % Search for the file ending with '_rlc_fitting.txt' in the folder
    file_pattern = fullfile(folder_path, '*_rlc_fitting.txt');
    file_info = dir(file_pattern);  % Get the file information
    
    % Check if the file exists
    if isempty(file_info)
        warning('No file found in folder %s matching the pattern "%s".', folder_path, file_pattern);
        continue;  % Skip to the next folder if no matching file is found
    end
    
    % Assuming there's only one file matching the pattern, get the first match
    filename = fullfile(folder_path, file_info(1).name);
    
    % Open the current file
    fid = fopen(filename, 'rt');
    if fid == -1
        error('File %s could not be opened', filename);
    end

    % Read the data from the file using fscanf
    data = fscanf(fid, '%f,');  % Read all the numbers as a 1D array
    fclose(fid);  % Close the file after reading

    % Check if the length of the data is divisible by 3
    if mod(length(data), 3) ~= 0
        % If not divisible by 3, remove extra elements from the end
        num_to_remove = mod(length(data), 3);
        data = data(1:end-num_to_remove);  % Remove excess elements
        % warning('Data in file %s was not divisible by 3. %d elements removed.', filename, num_to_remove);
    end

    % Reshape the data into 3 rows: frequency, magnitude, and phase
    data_reshaped = reshape(data, 3, []);  % Reshape into 3 rows: frequency, magnitude, phase
    
    % Unwrap the phase to avoid discontinuities
    unwrapped_phase = unwrap(data_reshaped(3, :));  % Unwrap phase data
    
    % Wrap the phase to the range [-pi, pi]
    wrapped_phase = mod(unwrapped_phase + pi, 2*pi) - pi;
    
    % Store the reshaped data in the appropriate cell array (one column per file)
    data_cell{1, i} = data_reshaped(1, :);  % Frequency
    data_cell{2, i} = data_reshaped(2, :);  % Magnitude
    data_cell{3, i} = wrapped_phase;         % Wrapped Phase to [-pi, pi]

    % % Optionally display the results for each file
    % fprintf('Data from file %s:\n', filename);
    % disp('Frequency:');
    % disp(data_cell{1, i});  % Frequency
    % disp('Magnitude:');
    % disp(data_cell{2, i});  % Magnitude
    % disp('Phase:');
    % disp(data_cell{3, i});  % Phase
end

%% Fitting
% Initialize the fit_cell, Z_fitted_cell, and components_cell to store the results for each file
fit_cell = cell(1, num_files);
Z_fitted_cell = cell(1, num_files);  % Cell array to store fitted Z values
components_cell = cell(1, num_files);  % Cell array to store best_popt (R, L, C) for each file

% Loop through each file in the data_cell
for i = 1:num_files
    % Get the frequency, magnitude, and phase data for the current file
    frequency = data_cell{1, i};
    magnitude = data_cell{2, i};
    phase = data_cell{3, i};
    
    % Call rlc_fit_re_im for the current file's data
    [best_model_name, results] = rlc_fit_re_im(frequency, magnitude, phase);
    
    % Store the results in fit_cell
    fit_cell{i} = struct('best_model_name', best_model_name, 'results', results);
    
    % Get the best model's popt (parameters)
    best_popt = results(best_model_name).popt;
    
    % Store best_popt (R, L, C) in components_cell
    components_cell{i} = best_popt;
    
    % Get the model function based on the best model name
    if strcmp(best_model_name, 'series')
        model_func = series_model;
    elseif strcmp(best_model_name, 'resistor')
        model_func = resistor_model;
    elseif strcmp(best_model_name, 'highres')
        model_func = high_value_res_model;
    else
        model_func = series_model;  % Default model
    end
    
    % Calculate the fitted impedance for the current file using the fitted parameters
    Z_fitted = model_func(best_popt, frequency);
    
    % Store the fitted impedance in Z_fitted_cell
    Z_fitted_cell{i} = Z_fitted;
    
    % Optionally display the results for each file
    fprintf('Results for file %d:\n', i);
    fprintf('Best model: %s\n', best_model_name);
    fprintf('R = %.3e Ohm\n', best_popt(1));
    fprintf('L = %.3e H\n', best_popt(2));
    fprintf('C = %.3e F\n', best_popt(3));
    fprintf('Sum of squared residuals: %.3e\n', results(best_model_name).ssq);
end

%% Assuming Z_values and data_cell have already been populated
% Separate the folders by type (R, C, L)
R_folders = {};
C_folders = {};
L_folders = {};

for i = 1:length(folder_list)
    Z_type = Z_values{i}{1};  % 'R', 'C', or 'L'
    if strcmp(Z_type, 'R')
        R_folders{end+1} = folder_list{i};
    elseif strcmp(Z_type, 'C')
        C_folders{end+1} = folder_list{i};
    elseif strcmp(Z_type, 'L')
        L_folders{end+1} = folder_list{i};
    end
end

% Function to extract the value between '_U_' and '_1k' for the legend
extract_impedance_value = @(folder_name) regexp(folder_name, 'U_(.*?)_1k', 'tokens', 'once');

% %% Display the final cell array for verification
% % disp('Final Data (Frequency, Magnitude, Phase for each file):');
% % disp(data_cell);
% 
% % If you want to visualize the data, you can create plots
% figure;
% subplot(2, 1, 1);
% semilogx(data_cell{1, 1}, 20*log10(data_cell{2, 1}), '-o');
% title('Magnitude (File 1)');
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% hold on;
% 
% % Get the best model and popt (parameters) for the first file
% best_model_name = fit_cell{1}.best_model_name;
% popt = fit_cell{1}.results(best_model_name).popt;
% 
% % Generate the fitted model for the current data (based on the model)
% if strcmp(best_model_name, 'series')
%     model_func = series_model;
% elseif strcmp(best_model_name, 'resistor')
%     model_func = resistor_model;
% elseif strcmp(best_model_name, 'highres')
%     model_func = high_value_res_model;
% else
%     model_func = series_model;  % Default model
% end
% 
% % Calculate fitted impedance for the current model
% Z_fitted = model_func(popt, data_cell{1, 1});
% 
% % Plot fitted magnitude as a dashed line
% semilogx(data_cell{1, 1}, 20*log10(abs(Z_fitted)), '--', ...
%     'DisplayName', ['Fitted ' best_model_name], 'LineWidth', 2);
% 
% subplot(2, 1, 2);
% semilogx(data_cell{1, 1}, data_cell{3, 1}, '-o');
% title('Phase (File 1)');
% xlabel('Frequency [Hz]');
% ylabel('Phase [rad]');
% hold on;
% 
% % Plot fitted phase as dashed line
% semilogx(data_cell{1, 1}, angle(Z_fitted), '--', ...
%     'DisplayName', ['Fitted ' best_model_name], 'LineWidth', 2);
% 
% legend('Location', 'southeast');
% 
% % Display the figure
% grid on;

%% Plot for R values
colors = linspecer(length(R_folders));
figure('Position', [100, 100, 1000, 800]);  % Increased the figure size
for i = 1:length(R_folders)
    folder_name = R_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Plot Magnitude (in dB)
    subplot(2, 1, 1);  % Subplot for magnitude
    semilogx(data_cell{1, folder_idx}, 20*log10(data_cell{2, folder_idx}), '-o', ...
        'DisplayName', impedance_value{1}, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
    
    % Plot the fitted model for the current file (from Z_fitted_cell) with a thinner line
    semilogx(data_cell{1, folder_idx}, 20*log10(abs(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value{1}], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line

    % Plot Phase (converted to degrees)
    subplot(2, 1, 2);  % Subplot for phase
    semilogx(data_cell{1, folder_idx}, rad2deg(data_cell{3, folder_idx}), '-o', ...
        'DisplayName', impedance_value{1}, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;

    % Plot the fitted model for phase (converted to degrees) with a thinner line
    semilogx(data_cell{1, folder_idx}, rad2deg(angle(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value{1}], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line
end
subplot(2, 1, 1);
title('Magnitude (R values)');
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

subplot(2, 1, 2);
title('Phase (R values)');
xlabel('Frequency [Hz]');
ylabel('Phase [degrees]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'R_values.png'));
end

%% Plot for C values
colors = linspecer(length(C_folders));
figure('Position', [100, 100, 1000, 800]);  % Increased the figure size
for i = 1:length(C_folders)
    folder_name = C_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Plot Magnitude (in dB)
    subplot(2, 1, 1);  % Subplot for magnitude
    semilogx(data_cell{1, folder_idx}, 20*log10(data_cell{2, folder_idx}), '-o', ...
        'DisplayName', impedance_value{1}, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
    
    % Plot the fitted model for the current file (from Z_fitted_cell) with a thinner line
    semilogx(data_cell{1, folder_idx}, 20*log10(abs(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value{1}], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line

    % Plot Phase (converted to degrees)
    subplot(2, 1, 2);  % Subplot for phase
    semilogx(data_cell{1, folder_idx}, rad2deg(data_cell{3, folder_idx}), '-o', ...
        'DisplayName', impedance_value{1}, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;

    % Plot the fitted model for phase (converted to degrees) with a thinner line
    semilogx(data_cell{1, folder_idx}, rad2deg(angle(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value{1}], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line
end
subplot(2, 1, 1);
title('Magnitude (C values)');
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

subplot(2, 1, 2);
title('Phase (C values)');
xlabel('Frequency [Hz]');
ylabel('Phase [degrees]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'C_values.png'));
end

%% Plot for L values
colors = linspecer(length(L_folders));
figure('Position', [100, 100, 1000, 800]);  % Increased the figure size
for i = 1:length(L_folders)
    folder_name = L_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Plot Magnitude (in dB)
    subplot(2, 1, 1);  % Subplot for magnitude
    semilogx(data_cell{1, folder_idx}, 20*log10(data_cell{2, folder_idx}), '-o', ...
        'DisplayName', impedance_value{1}, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
    
    % Plot the fitted model for the current file (from Z_fitted_cell) with a thinner line
    semilogx(data_cell{1, folder_idx}, 20*log10(abs(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value{1}], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line

    % Plot Phase (converted to degrees)
    subplot(2, 1, 2);  % Subplot for phase
    semilogx(data_cell{1, folder_idx}, rad2deg(data_cell{3, folder_idx}), '-o', ...
        'DisplayName', impedance_value{1}, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;

    % Plot the fitted model for phase (converted to degrees) with a thinner line
    semilogx(data_cell{1, folder_idx}, rad2deg(angle(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value{1}], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line
end
subplot(2, 1, 1);
title('Magnitude (L values)');
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

subplot(2, 1, 2);
title('Phase (L values)');
xlabel('Frequency [Hz]');
ylabel('Phase [degrees]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'L_values.png'));
end

%% Plot for R values (Magnitude Error in Percentage)
colors = linspecer(length(R_folders));
figure('Position', [100, 100, 800, 600]);  % Increased the figure size
for i = 1:length(R_folders)
    folder_name = R_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Calculate magnitude error in percentage
    fitted_Z = log10(abs(Z_fitted_cell{folder_idx}));  % Get the fitted impedance
    measured_magnitude = log10(data_cell{2, folder_idx});  % Measured magnitude

    percentage_error = (abs((measured_magnitude - fitted_Z) ./ ...
            fitted_Z)) * 100;  % Calculate magnitude error in percentage
    
    % Calculate the average error for the legend
    avg_error = mean(percentage_error);  % Average percentage error
    
    % Plot Magnitude Error (in percentage)
    semilogx(data_cell{1, folder_idx}, percentage_error, '-o', ...
        'DisplayName', sprintf('%s - Avg Error: %.2f%%', impedance_value{1}, avg_error), ...
        'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
end

title('Magnitude Error (R values)');
xlabel('Frequency [Hz]');
ylabel('Magnitude Error [%]');
legend('Location', 'northeast');
set(gca, 'FontSize', 12);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'R_values_error.png'));
end

%% Plot for C values (Magnitude Error in Percentage)
colors = linspecer(length(C_folders));
figure('Position', [100, 100, 800, 600]);  % Increased the figure size
for i = 1:length(C_folders)
    folder_name = C_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Calculate magnitude error in percentage
    fitted_Z = log10(abs(Z_fitted_cell{folder_idx}));  % Get the fitted impedance
    measured_magnitude = log10(data_cell{2, folder_idx});  % Measured magnitude
    
    percentage_error = (abs((measured_magnitude - fitted_Z) ./ ...
            fitted_Z)) * 100;  % Calculate magnitude error in percentage

    % Calculate the average error for the legend
    avg_error = mean(percentage_error);  % Average percentage error
    
    % Plot Magnitude Error (in percentage)
    semilogx(data_cell{1, folder_idx}, percentage_error, '-o', ...
        'DisplayName', sprintf('%s - Avg Error: %.2f%%', impedance_value{1}, avg_error), ...
        'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
end

title('Magnitude Error (C values)');
xlabel('Frequency [Hz]');
ylabel('Magnitude Error [%]');
legend('Location', 'northeast');
set(gca, 'FontSize', 12);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'C_values_error.png'));
end

%% Plot for L values (Magnitude Error in Percentage)
colors = linspecer(length(L_folders));
figure('Position', [100, 100, 800, 600]);  % Increased the figure size
for i = 1:length(L_folders)
    folder_name = L_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Calculate magnitude error in percentage
    fitted_Z = log10(abs(Z_fitted_cell{folder_idx}));  % Get the fitted impedance
    measured_magnitude = log10(data_cell{2, folder_idx});  % Measured magnitude

    percentage_error = (abs((measured_magnitude - fitted_Z) ./ ...
            fitted_Z)) * 100;  % Calculate magnitude error in percentage
    
    % Calculate the average error for the legend
    avg_error = mean(percentage_error);  % Average percentage error
    
    % Plot Magnitude Error (in percentage)
    semilogx(data_cell{1, folder_idx}, percentage_error, '-o', ...
        'DisplayName', sprintf('%s - Avg Error: %.2f%%', impedance_value{1}, avg_error), ...
        'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
end

title('Magnitude Error (L values)');
xlabel('Frequency [Hz]');
ylabel('Magnitude Error [%]');
legend('Location', 'northeast');
set(gca, 'FontSize', 12);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'L_values_error.png'));
end


%% Average Error vs Frequency (Magnitude)
avg_magnitude_error = zeros(1, length(data_cell{1, 1}));  % Initialize array for average error
for i = 1:num_files
    % Calculate error for each file and accumulate
    magnitude_error = (abs(20 * log10(data_cell{2, i}) - 20 * log10(abs(Z_fitted_cell{i}))) ./ ...
        (20 * log10(abs(Z_fitted_cell{i})))) * 100;  % Calculate magnitude error in percentage
    avg_magnitude_error = avg_magnitude_error + magnitude_error;
end
avg_magnitude_error = avg_magnitude_error / length(data_cell{1, 1});  % Average over all files

figure('Position', [100, 100, 800, 600]);  % Increased the figure size
semilogx(data_cell{1, 1}, avg_magnitude_error, '-o', 'LineWidth', 2, 'Color', 'r', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
title('Average Magnitude Error vs Frequency');
xlabel('Frequency [Hz]');
ylabel('Average Magnitude Error [%]');
grid on;
set(gca, 'FontSize', 12);  % Set smaller font size for legend

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'error_vs_frequency.png'));
end

%% Average Error vs Impedance
sample_error = zeros(1, num_files);  % Initialize array for average impedance error
Z_series_all = zeros(1, num_files);  % To store the series impedance for each file
sample_freq = 4500;

for i = 1:num_files
    % Get the best model's parameters (R, L, C) from Z_fitted_cell
    best_popt = components_cell{i};
    
    % Extract the R, L, and C values from the best fit model parameters
    R = best_popt(1);
    L = best_popt(2);
    C = best_popt(3);


    % Calculate impedance (Z) for the DUT (R + jωL + 1/jωC)
    omega = 2 * pi * sample_freq;  % Angular frequency at sampling frequency
    Z_series = R + 1i * omega * L + 1 ./ (1i * omega * C);  % Impedance of the series RLC circuit

    Z_series_all(i) = abs(Z_series);  % Store the magnitude of the fitted impedance for each file

    % Find the index of the closest frequency to sample_freq
    frequency = data_cell{1, i};
    [~, idx] = min(abs(frequency - sample_freq));  % Get index of closest frequency to sample_freq
    
    % Calculate impedance (Z) for the DUT (R + jωL + 1/jωC)
    omega = 2 * pi * sample_freq;  % Angular frequency at sampling frequency
    Z_series = R + 1i * omega * L + 1 ./ (1i * omega * C);  % Impedance of the series RLC circuit

    % Calculate magnitude error in percentage at the sampling frequency (or closest frequency)
    fitted_Z = log10(abs(Z_fitted_cell{i}(idx)));  % Get the fitted impedance at the closest frequency
    measured_magnitude = log10(data_cell{2, i}(idx));  % Measured magnitude at the closest frequency

    sample_error(i) = (abs((measured_magnitude - fitted_Z) ./ fitted_Z)) * 100;  % Calculate magnitude error in percentage

end

% Sort by Z_series_all (impedance magnitude)
[Z_series_all_sorted, sort_idx] = sort(Z_series_all);
sample_error_sorted = sample_error(sort_idx);  % Sort sample_error according to the sorted impedance

% Exclude Z_series_all_sorted greater than 10^6 and corresponding sample_error
threshold = 10^6;
filtered_idx = Z_series_all_sorted <= threshold;

Z_series_all_sorted = Z_series_all_sorted(filtered_idx);
sample_error_sorted = sample_error_sorted(filtered_idx);

figure('Position', [100, 100, 800, 600]);  % Increased the figure size
semilogx(Z_series_all_sorted, sample_error_sorted, '-o', 'LineWidth', 2, 'Color', 'b', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
title('Average Magnitude Error vs Impedance (at 4.5kHz)');
xlabel('log|Z| [Ohm]');
ylabel('Average Magnitude Error [%]');
grid on;
set(gca, 'FontSize', 12);  % Set smaller font size for legend

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'error_vs_impedance.png'));
end

%% Calculate and Plot Quality Factor (Q)
Q_factor_series = zeros(1, num_files);  % Initialize array for Quality Factor
Q_factor_parallel = zeros(1, num_files);  % Initialize array for Quality Factor

for i = 1:num_files
    % Get the best model's parameters (R, L, C) from components_cell
    best_popt = components_cell{i};
    
    % Extract the R, L, and C values from the best fit model parameters
    R = best_popt(1);
    L = best_popt(2);
    C = best_popt(3);
    
    % Calculate the Quality Factor (Q) for the series RLC circuit
    Q_factor_series(i) = sqrt(L / C) / R;
    % Q_factor_parallel(i) = sqrt(C / R) * R; % Uncomment if you want parallel Q-factor
end

% Sort by impedance
[Z_series_filtered, sort_idx] = sort(Z_series_all_sorted);
Q_factor_series_filtered = Q_factor_series(sort_idx);  % Sorting impedance based on Q-factor order
% Q_factor_parallel_filtered = Q_factor_parallel(sort_idx);  % Sorting impedance based on Q-factor order

% Now, we need to map the sorted impedance back to the folder names
folder_names_sorted = folder_list(sort_idx);  % Get the folder names corresponding to the sorted impedance

% Prepare color map for scatter
colors = linspecer(length(folder_names_sorted));  % Choose a color map (jet is a good default)

% Create a scatter plot for Quality Factor vs Impedance
figure('Position', [100, 100, 800, 600]);  % Increased the figure size
hold on;

% Loop through and plot each data point with a unique color
for i = 1:length(Z_series_filtered)
    % Extract the impedance value for the current file
    Z_val = Z_series_filtered(i);
    
    % Extract the folder name from the sorted folder names
    folder_name = folder_names_sorted{i};
    
    % Extract the impedance value for legend (just like previously done)
    impedance_value = extract_impedance_value(folder_name);
    
    % Scatter plot each point with a different color
    scatter(Z_val, Q_factor_series_filtered(i), 100, colors(i,:), 'filled', ...
        'DisplayName', impedance_value{1});  % Use impedance_value as the label in legend
end

% Set x-axis to logarithmic scale
set(gca, 'XScale', 'log');
set(gca, 'YScale', 'log');

% Add title and labels
title('Quality Factor vs Impedance (at 4.5kHz)');
xlabel('log|Z| [Ohm]');
ylabel('log(Q)');
grid on;
set(gca, 'FontSize', 12);  % Set smaller font size for legend

% Add legend to the plot
legend('Location', 'best');  % Show all legend entries

% Save the figure as PNG if save_plots is true
if save_plots
    saveas(gcf, fullfile('figures_processed', 'Q_factor.png'));
end

%% RMSE vs Impedance for All Files (Using Z_series) with Threshold
colors = linspecer(length(folder_list));  % Generate a color map

figure('Position', [100, 100, 800, 600]);  % Increased the figure size
hold on;

sample_freq = 4500;  % Define the sample frequency (4500 Hz)
threshold = 10^7;  % Define the threshold impedance (example: 10^6 Ohms)

for i = 1:num_files
    folder_name = folder_list{i};  % Current folder name
    
    % Extract impedance value for the legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Get the best model's parameters (R, L, C) from components_cell
    best_popt = components_cell{i};  
    R = best_popt(1);  % Resistance
    L = best_popt(2);  % Inductance
    C = best_popt(3);  % Capacitance

    % Calculate impedance (Z) for the DUT at the sample frequency (R + jωL + 1/jωC)
    omega = 2 * pi * sample_freq;  % Angular frequency at sample frequency
    Z_series = R + 1i * omega * L + 1 ./ (1i * omega * C);  % Impedance of the series RLC circuit
    Z_series_magnitude = abs(Z_series);  % Impedance magnitude for the current file

    % Check if impedance exceeds the threshold
    if Z_series_magnitude > threshold
        continue;  % Skip plotting this point if the impedance exceeds the threshold
    end

    % Get the fitted impedance for the current file
    fitted_Z = abs(Z_fitted_cell{i});
    
    % Calculate RMSE at each frequency (log scale)
    log_diff = log10(fitted_Z) - log10(Z_series_magnitude);  % Difference in log10 scale
    rmse = sqrt(mean(log_diff.^2));  % RMSE calculation
    
    % Plot RMSE vs impedance (log scale)
    semilogx(Z_series_magnitude, repmat(rmse, size(Z_series_magnitude)), 'o', ...
        'DisplayName', impedance_value{1}, ...
        'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 8, 'MarkerFaceColor', colors(i, :));
    hold on;
end

% Set x-axis to logarithmic scale for impedance
set(gca, 'XScale', 'log');
xlabel('Impedance [Ohm]');
ylabel('RMSE');
title('RMSE vs Impedance (at 4.5kHz)');
legend('Location', 'best');
set(gca, 'FontSize', 12);  % Set font size for readability
grid on;

% Save the figure as PNG if save_plots is true
if save_plots
    saveas(gcf, fullfile('figures_processed', 'RMSE_vs_Impedance.png'));
end
