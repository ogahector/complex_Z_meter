%% Define parameters
time_step = 8e-6;
voltage_step = 1e-3;
max_voltage = 4.095;
num_steps = round(max_voltage / voltage_step) + 1;
pause_duration = 5e-6; % Additional time for pause

% Initialize time and voltage arrays
time_values = [];
voltage_values = [];

%% Generate the step waveform with pauses
current_time = 0;
for i = 0:num_steps-1
    voltage = i * voltage_step;  % Compute voltage level
    
    % Add step point
    time_values = [time_values; current_time];  % Time for this step
    voltage_values = [voltage_values; voltage]; % Voltage level

    % Add pause point (same voltage)
    current_time = current_time + time_step;
    time_values = [time_values; current_time];
    voltage_values = [voltage_values; voltage];

    % Increment time for next step
    current_time = current_time + pause_duration;
end

%% Create table for saving
data = table(time_values, voltage_values);

% Save to text file
file_name = 'vin_step_waveform.txt';
writetable(data, file_name, 'Delimiter', '\t', 'WriteVariableNames', false);

disp(['File saved as: ', file_name]);
