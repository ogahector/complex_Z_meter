close all; clc; clear
save_plots = true;  % Set this to true to save the plots, or false to not save

% List of folders to search
folder_list = { 'measurement_data/D20250326_E02_U_R0_L1mH_C1uF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L10mH_C1uF_PAR_1k', ...
                'measurement_data/D20250326_E02_U_R0_L10mH_C1uF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L100mH_C10nF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L100mH_C100nF_PAR_1k',...
                'measurement_data/D20250326_E02_U_R0_L4700uH_C1uF_PAR_1k', ...
                'measurement_data/D20250326_E02_U_R0_L4700uH_C1uF_SER_1k', ...
                'measurement_data/D20250326_E02_U_R0_L4700uH_C100nF_SER_1k'};
num_files = length(folder_list);

% Series model (RLC all in series)
series_model = @(params, freq) (params(1) + 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3)));
% Resistor model (RL in series, with C in shunt)
resistor_model = @(params, freq) 1 ./ (1i .* 2 .* pi .* freq .* params(3) + 1 ./ (params(1) + 1i .* 2 .* pi .* freq .* params(2)));
% High value resistor model
high_value_res_model = @(params, freq) 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3) + 1 ./ params(1));
% Parallel model (RLC all in parallel)
parallel_model = @(params, freq) 1 ./ (params(1) + 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3)));

%% Initialize cell arrays to store data
data_cell = cell(3, length(folder_list));  % Frequency, magnitude, and phase for each file
components_cell = cell(1, length(folder_list));  % To store the best parameters (R, L, C) for each file
Z_fitted_cell = cell(1, length(folder_list));  % Cell array to store fitted Z values
fit_cell = cell(1, length(folder_list));  % Store the fit results
legend_cell = cell(1, length(folder_list));  % Store the legend entries

% Loop through each folder to process the data
for i = 1:length(folder_list)
    folder_path = folder_list{i};
    
    % Extract L, and C values from the folder name using the template regex
    expr = 'L(\d+)([a-zA-Z]+)_C(\d+)([a-zA-Z]+)';  % Regex pattern to extract L and C values
    match = regexp(folder_path, expr, 'tokens', 'once');
    
    if isempty(match)
        warning('No valid L and C values found in folder: %s', folder_path);
        continue;  % Skip folders without L and C values
    end
    
    % Extract L and C values from the match
    L_value = str2double(match{1});
    L_unit = match{2};
    C_value = str2double(match{3});
    C_unit = match{4};

    % Save the legend entry (L and C values for the legend)
    legend_cell{i} = sprintf('%g%s %g%s', L_value, L_unit, C_value, C_unit);
    
    % Convert L and C values to standard units
    if strcmp(L_unit, 'mH')
        L_value = L_value * 1e-3;  % Convert millihenry to henry
    elseif strcmp(L_unit, 'uH')
        L_value = L_value * 1e-6;  % Convert microhenry to henry
    elseif strcmp(L_unit, 'kH')
        L_value = L_value * 1e3;   % Convert kilohenry to henry
    end
    
    if strcmp(C_unit, 'nF')
        C_value = C_value * 1e-9;  % Convert nanofarad to farad
    elseif strcmp(C_unit, 'uF')
        C_value = C_value * 1e-6;  % Convert microfarad to farad
    elseif strcmp(C_unit, 'mF')
        C_value = C_value * 1e-3;  % Convert millifarad to farad
    elseif strcmp(C_unit, 'kF')
        C_value = C_value * 1e3;   % Convert kilofarad to farad
    end
    
    % Determine if the configuration is series or parallel
    if contains(folder_path, 'SER')
        config_type = 'SER';  % Series model
    elseif contains(folder_path, 'PAR')
        config_type = 'PAR';  % Parallel model
    else
        continue;  % Skip folders that don't fit the expected naming pattern
    end
    
    % Search for the file ending with '_rlc_fitting.txt' in the folder
    file_pattern = fullfile(folder_path, '*_rlc_fitting.txt');
    file_info = dir(file_pattern);  % Get the file information
    
    if isempty(file_info)
        warning('No file found in folder %s matching the pattern "%s".', folder_path, file_pattern);
        continue;  % Skip to the next folder if no matching file is found
    end
    
    % Assuming only one file, use the first match
    filename = fullfile(folder_path, file_info(1).name);
    
    % Read the data from the file
    fid = fopen(filename, 'rt');
    data = fscanf(fid, '%f,');
    fclose(fid);
    
    if mod(length(data), 3) ~= 0
        data = data(1:end-mod(length(data), 3));  % Adjust data size
    end

    data_reshaped = reshape(data, 3, []);  
    data_cell{1, i} = data_reshaped(1, :);  % Frequency
    data_cell{2, i} = data_reshaped(2, :);  % Magnitude
    data_cell{3, i} = unwrap(data_reshaped(3, :));  % Phase
    
    % Pass L_value and C_value as initial guesses when calling rlc_fit_re_im
    [best_model_name, results] = rlc_fit_re_im(data_cell{1, i}, data_cell{2, i}, data_cell{3, i}, ...
                                               L_value, C_value);
    fit_cell{i} = struct('best_model_name', best_model_name, 'results', results);
    
    % Store the best fitting parameters (R, L, C) for the current file
    best_popt = results(best_model_name).popt;
    components_cell{i} = best_popt;
    
    % Get the model function based on the best model name
    if strcmp(best_model_name, 'series')
        model_func = series_model;
    elseif strcmp(best_model_name, 'resistor')
        model_func = resistor_model;
    elseif strcmp(best_model_name, 'highres')
        model_func = high_value_res_model;
    elseif strcmp(best_model_name, 'parallel')
        model_func = parallel_model;
    end
    
    % Calculate the fitted impedance using the selected model
    Z_fitted = model_func(best_popt, data_cell{1, i});
    Z_fitted_cell{i} = Z_fitted;

     % Print the results
    fprintf('Results for folder: %d\n', i);
    fprintf('Best model: %s\n', best_model_name);
    fprintf('R = %.3e Ohm\n', best_popt(1));
    fprintf('L = %.3e H\n', best_popt(2));
    fprintf('C = %.3e F\n', best_popt(3));
    fprintf('Sum of squared residuals: %.3e\n\n', results(best_model_name).ssq);
end

% Function to extract the corresponding legend entry for a given folder
extract_impedance_value = @(folder_name) legend_cell{find(strcmp(folder_list, folder_name))};

%% Separate the folders into SER and PAR types
SER_folders = {};  % Series folder list
PAR_folders = {};  % Parallel folder list

for i = 1:length(folder_list)
    if contains(folder_list{i}, 'SER')
        SER_folders{end+1} = folder_list{i};  % Add to series list
    elseif contains(folder_list{i}, 'PAR')
        PAR_folders{end+1} = folder_list{i};  % Add to parallel list
    end
end

%% Plot for SER files
colors = linspecer(length(SER_folders));
figure('Position', [100, 100, 1000, 800]);  % Increased the figure size
for i = 1:length(SER_folders)
    folder_name = SER_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Plot Magnitude (in dB)
    subplot(2, 1, 1);  % Subplot for magnitude
    semilogx(data_cell{1, folder_idx}, 20*log10(data_cell{2, folder_idx}), '-o', ...
        'DisplayName', impedance_value, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
    
    % Plot the fitted model for the current file (from Z_fitted_cell) with a thinner line
    semilogx(data_cell{1, folder_idx}, 20*log10(abs(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line

    % Plot Phase (converted to degrees)
    subplot(2, 1, 2);  % Subplot for phase
    semilogx(data_cell{1, folder_idx}, rad2deg(data_cell{3, folder_idx}), '-o', ...
        'DisplayName', impedance_value, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;

    % Plot the fitted model for phase (converted to degrees) with a thinner line
    semilogx(data_cell{1, folder_idx}, rad2deg(angle(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line
end
subplot(2, 1, 1);
title('Magnitude (Series RLC)');
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

subplot(2, 1, 2);
title('Phase (Series RLC)');
xlabel('Frequency [Hz]');
ylabel('Phase [degrees]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'SER_values.png'));
end

%% Plot for PAR files
colors = linspecer(length(PAR_folders));
figure('Position', [100, 100, 1000, 800]);  % Increased the figure size
for i = 1:length(PAR_folders)
    folder_name = PAR_folders{i};
    % Find corresponding folder index
    folder_idx = find(strcmp(folder_list, folder_name));
    
    % Extract impedance value for legend
    impedance_value = extract_impedance_value(folder_name);
    
    % Plot Magnitude (in dB)
    subplot(2, 1, 1);  % Subplot for magnitude
    semilogx(data_cell{1, folder_idx}, 20*log10(data_cell{2, folder_idx}), '-o', ...
        'DisplayName', impedance_value, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
    
    % Plot the fitted model for the current file (from Z_fitted_cell) with a thinner line
    semilogx(data_cell{1, folder_idx}, 20*log10(abs(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line

    % Plot Phase (converted to degrees)
    subplot(2, 1, 2);  % Subplot for phase
    semilogx(data_cell{1, folder_idx}, rad2deg(data_cell{3, folder_idx}), '-o', ...
        'DisplayName', impedance_value, 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;

    % Plot the fitted model for phase (converted to degrees) with a thinner line
    semilogx(data_cell{1, folder_idx}, rad2deg(angle(Z_fitted_cell{folder_idx})), '--', ...
        'DisplayName', ['Fitted ' impedance_value], 'LineWidth', 1, 'Color', colors(i, :));  % Thinner line
end
subplot(2, 1, 1);
title('Magnitude (Parallel RLC)');
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

subplot(2, 1, 2);
title('Phase (Parallel RLC)');
xlabel('Frequency [Hz]');
ylabel('Phase [degrees]');
legend('Location', 'southeast');
set(gca, 'FontSize', 9);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'PAR_values.png'));
end

%% Plot for SER files (Magnitude Error in Percentage)
colors = linspecer(length(SER_folders));
figure('Position', [100, 100, 800, 600]);  % Increased the figure size
for i = 1:length(SER_folders)
    folder_name = SER_folders{i};
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
        'DisplayName', sprintf('%s - Avg Error: %.2f%%', impedance_value, avg_error), ...
        'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
end

title('Magnitude Error (Series RLC)');
xlabel('Frequency [Hz]');
ylabel('Magnitude Error [%]');
legend('Location', 'northeast');
set(gca, 'FontSize', 12);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'SER_values_error.png'));
end

%% Plot for PAR files (Magnitude Error in Percentage)
colors = linspecer(length(PAR_folders));
figure('Position', [100, 100, 800, 600]);  % Increased the figure size
for i = 1:length(PAR_folders)
    folder_name = PAR_folders{i};
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
        'DisplayName', sprintf('%s - Avg Error: %.2f%%', impedance_value, avg_error), ...
        'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 3, 'MarkerFaceColor', colors(i, :));
    hold on;
end

title('Magnitude Error (Parallel RLC)');
xlabel('Frequency [Hz]');
ylabel('Magnitude Error [%]');
legend('Location', 'northeast');
set(gca, 'FontSize', 12);  % Set smaller font size for legend
grid on;

if save_plots
    % Save the figure as PNG in the 'figures_processed' subfolder
    saveas(gcf, fullfile('figures_processed', 'PAR_values_error.png'));
end

%% Average Error vs Frequency (Magnitude) for SER and PAR files (Single Plot)
avg_magnitude_error = zeros(1, length(data_cell{1, 1}));  % Initialize array for average error
avg_magnitude_error_SER = zeros(1, length(data_cell{1, 1}));  % Initialize array for SER files
avg_magnitude_error_PAR = zeros(1, length(data_cell{1, 1}));  % Initialize array for PAR files

for i = 1:num_files
    % Calculate error for each file and accumulate for SER files
    if contains(folder_list{i}, 'SER')
        magnitude_error_SER = (abs(20 * log10(data_cell{2, i}) - 20 * log10(abs(Z_fitted_cell{i}))) ./ ...
            (20 * log10(abs(Z_fitted_cell{i})))) * 100;  % Calculate magnitude error in percentage
        avg_magnitude_error_SER = avg_magnitude_error_SER + magnitude_error_SER;
    end
    
    % Calculate error for each file and accumulate for PAR files
    if contains(folder_list{i}, 'PAR')
        magnitude_error_PAR = (abs(20 * log10(data_cell{2, i}) - 20 * log10(abs(Z_fitted_cell{i}))) ./ ...
            (20 * log10(abs(Z_fitted_cell{i})))) * 100;  % Calculate magnitude error in percentage
        avg_magnitude_error_PAR = avg_magnitude_error_PAR + magnitude_error_PAR;
    end
end

% Average over all files for SER and PAR
avg_magnitude_error_SER = avg_magnitude_error_SER / length(SER_folders);
avg_magnitude_error_PAR = avg_magnitude_error_PAR / length(PAR_folders);

% Calculate total average magnitude error for all files (both SER and PAR)
avg_magnitude_error = avg_magnitude_error_SER + avg_magnitude_error_PAR;

% Plot for both SER and PAR files (combined)
figure('Position', [100, 100, 800, 600]);  
semilogx(data_cell{1, 1}, avg_magnitude_error_SER, '-o', 'LineWidth', 2, 'Color', 'r', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
hold on;
semilogx(data_cell{1, 1}, avg_magnitude_error_PAR, '-o', 'LineWidth', 2, 'Color', 'b', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
title('Average Magnitude Error vs Frequency');
xlabel('Frequency [Hz]');
ylabel('Average Magnitude Error [%]');
legend({'Series RLC', 'Parallel RLC'}, 'Location', 'northeast');
grid on;
set(gca, 'FontSize', 12);  

if save_plots
    saveas(gcf, fullfile('figures_processed', 'error_vs_frequency_SER_PAR.png'));
end

%% Average Error vs Impedance for SER and PAR files (Single Plot)
sample_error = zeros(1, num_files);  % Initialize array for average impedance error
Z_all = zeros(1, num_files);  % To store the impedance (both series and parallel) for each file
sample_freq = 4500;  % Sample frequency at 4500 Hz

for i = 1:num_files
    % Get the best model's parameters (R, L, C) from Z_fitted_cell for SER files
    if contains(folder_list{i}, 'SER')
        best_popt_SER = components_cell{i};
        R = best_popt_SER(1);
        L = best_popt_SER(2);
        C = best_popt_SER(3);

        % Find the index of the closest frequency to sample_freq
        frequency = data_cell{1, i};
        [~, idx] = min(abs(frequency - sample_freq));  % Get index of closest frequency to sample_freq

        % Calculate impedance (SER: R + jωL + 1/jωC)
        omega = 2 * pi * sample_freq;  % Angular frequency at sample frequency
        Z_series = R + 1i * omega * L + 1 ./ (1i * omega * C);  % Impedance of the series RLC circuit
        Z_all(i) = abs(Z_series);  % Store the magnitude of the fitted impedance for each SER file

        % Calculate magnitude error in percentage at the sample frequency (or closest frequency)
        fitted_Z_SER = log10(abs(Z_fitted_cell{i}(idx)));  % Get the fitted impedance at the closest frequency
        measured_magnitude_SER = log10(data_cell{2, i}(idx));  % Measured magnitude at the closest frequency

        sample_error(i) = (abs((measured_magnitude_SER - fitted_Z_SER) ./ fitted_Z_SER)) * 100;  % Calculate magnitude error in percentage
    end

    % Get the best model's parameters (R, L, C) from Z_fitted_cell for PAR files
    if contains(folder_list{i}, 'PAR')
        best_popt_PAR = components_cell{i};
        R = best_popt_PAR(1);
        L = best_popt_PAR(2);
        C = best_popt_PAR(3);

        % Find the index of the closest frequency to sample_freq
        frequency = data_cell{1, i};
        [~, idx] = min(abs(frequency - sample_freq));  % Get index of closest frequency to sample_freq

        % Calculate impedance (PAR: 1 / (R + jωL + 1/jωC))
        omega = 2 * pi * sample_freq;  % Angular frequency at sample frequency
        Z_parallel = 1 / (R + 1i * omega * L + 1 / (1i * omega * C));  % Impedance of the parallel RLC circuit
        Z_all(i) = abs(Z_parallel);  % Store the magnitude of the fitted impedance for each PAR file

        % Calculate magnitude error in percentage at the sample frequency (or closest frequency)
        fitted_Z_PAR = log10(abs(Z_fitted_cell{i}(idx)));  % Get the fitted impedance at the closest frequency
        measured_magnitude_PAR = log10(data_cell{2, i}(idx));  % Measured magnitude at the closest frequency

        sample_error(i) = (abs((measured_magnitude_PAR - fitted_Z_PAR) ./ fitted_Z_PAR)) * 100;  % Calculate magnitude error in percentage
    end
end

% Sort by Z_all (impedance magnitude)
[Z_all_sorted, sort_idx] = sort(Z_all);
sample_error_sorted = sample_error(sort_idx);  % Sort sample_error according to the sorted impedance

% Exclude Z_all_sorted greater than 10^6 and corresponding sample_error
threshold = 10^6;
filtered_idx = Z_all_sorted <= threshold;

Z_all_sorted = Z_all_sorted(filtered_idx);
sample_error_sorted = sample_error_sorted(filtered_idx);

% Plot for both SER and PAR files (combined in a single plot)
figure('Position', [100, 100, 800, 600]);  
semilogx(Z_all_sorted, sample_error_sorted, '-o', 'LineWidth', 2, 'Color', 'g', 'MarkerSize', 3, 'MarkerFaceColor', 'g');
title('Average Magnitude Error vs Impedance');
xlabel('log|Z| [Ohm]');
ylabel('Average Magnitude Error [%]');
grid on;
set(gca, 'FontSize', 12);  

if save_plots
    saveas(gcf, fullfile('figures_processed', 'error_vs_impedance_SER_PAR.png'));
end

% %% Calculate RMSE for all files (combined for SER and PAR)
% colors = linspecer(length(folder_list));  % Generate a color map
% 
% figure('Position', [100, 100, 800, 600]);  
% hold on;
% threshold = 10^6;  % Impedance threshold for plotting
% 
% for i = 1:length(folder_list)
%     folder_name = folder_list{i};
%     folder_idx = i;  % Current folder index
% 
%     % Extract the model parameters for the current folder
%     best_popt = components_cell{folder_idx};
%     R = best_popt(1);
%     L = best_popt(2);
%     C = best_popt(3);
% 
%     % Calculate impedance (using either series or parallel model)
%     omega = 2 * pi * 4500;  % Frequency of 4500 Hz
%     if contains(folder_name, 'SER')  % For Series configuration
%         Z_series = R + 1i * omega * L + 1 / (1i * omega * C);
%         Z_series_mag = abs(Z_series);
%     elseif contains(folder_name, 'PAR')  % For Parallel configuration
%         Z_parallel = 1 / (R + 1i * omega * L + 1 / (1i * omega * C));
%         Z_parallel_mag = abs(Z_parallel);
%     end
% 
%     % Check if impedance exceeds the threshold
%     if Z_series_mag > threshold
%         continue;
%     end
% 
%     % Calculate RMSE for the current file (with respect to the series model)
%     fitted_Z = abs(Z_fitted_cell{folder_idx});
%     if contains(folder_name, 'SER')
%         rmse = sqrt(mean((log10(fitted_Z) - log10(Z_series_mag)).^2));  % RMSE for series
%     elseif contains(folder_name, 'PAR')
%         rmse = sqrt(mean((log10(fitted_Z) - log10(Z_parallel_mag)).^2));  % RMSE for parallel
%     end
% 
%     % Plot RMSE vs impedance
%     semilogx(Z_series_mag, rmse, 'o', 'DisplayName', extract_impedance_value(folder_name), 'LineWidth', 2, 'Color', colors(i, :), 'MarkerSize', 8, 'MarkerFaceColor', colors(i, :));
%     hold on;
% end
% 
% set(gca, 'XScale', 'log');
% xlabel('Impedance [Ohm]');
% ylabel('RMSE');
% title('RMSE vs Impedance (Series and Parallel)');
% legend('Location', 'best');
% grid on;
% 
% if save_plots
%     saveas(gcf, fullfile('figures_processed', 'RMSE_vs_Impedance_SER_PAR.png'));
% end
% 
% %% Calculate Quality Factor (Q) for all files
% Q_factor_series = zeros(1, length(folder_list));
% Q_factor_parallel = zeros(1, length(folder_list));
% 
% for i = 1:length(folder_list)
%     best_popt = components_cell{i};
%     R = best_popt(1);
%     L = best_popt(2);
%     C = best_popt(3);
% 
%     % Calculate Quality Factor based on series or parallel model
%     if contains(folder_list{i}, 'SER')
%         Q_factor_series(i) = sqrt(L / C) / R;  % Series formula
%     elseif contains(folder_list{i}, 'PAR')
%         Q_factor_parallel(i) = sqrt(C / R) * R;  % Parallel formula
%     end
% end
% 
% % Plot Quality Factor vs Impedance
% figure('Position', [100, 100, 800, 600]);  
% hold on;
% for i = 1:length(folder_list)
%     folder_name = folder_list(i);
%     if contains(folder_list{i}, 'SER')
%         scatter(Z_series_mag, Q_factor_series(i), 100, 'filled', 'DisplayName', extract_impedance_value(folder_name));
%     elseif contains(folder_list{i}, 'PAR')
%         scatter(Z_series_mag, Q_factor_parallel(i), 100, 'filled', 'DisplayName', extract_impedance_value(folder_name));
%     end
%     hold on;
% end
% 
% set(gca, 'XScale', 'log');
% set(gca, 'YScale', 'log');
% xlabel('Impedance [Ohm]');
% ylabel('Quality Factor (Q)');
% title('Quality Factor vs Impedance for all configurations');
% legend('Location', 'best');
% grid on;
% 
% if save_plots
%     saveas(gcf, fullfile('figures_processed', 'Q_factor_SER_PAR.png'));
% end
