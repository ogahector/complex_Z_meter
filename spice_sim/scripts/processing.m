clc; clear;
close all

data = import('IV_real_initialTest00');

%% Processing

plotwfs(data);

setups = data.keys();
for i = 1:length(setups)
    SETUP = setups{i};
    wf = data(SETUP);
    dut = (wf.V2 ./ abs(wf.V2 - wf.V1)) .* wf.Rref;

     figure;
        set(gcf, 'Position', [50*i 50*i 800 500]);
        hold on;
        plot(wf.time, dut, '-', 'LineWidth', 2);
        hold off;
        xlabel('Time [s]');
        ylabel('DUT');

    disp(mean(dut));
end


%% ---- PLOT WAVEFORMS ----
function plotwfs(data)
    colors = linspecer(2);
    
    setups = data.keys();
    for i = 1:length(setups)
        SETUP = setups{i};
        wf = data(SETUP);
    
        figure;
        set(gcf, 'Position', [50*i 50*i 800 500]);
        hold on;
        plot(wf.time, wf.V1, '-', 'Color', colors(1, :), 'LineWidth', 2);
        plot(wf.time, wf.V2, '-', 'Color', colors(2, :), 'LineWidth', 2);
        hold off;
        title(['Waveforms (' SETUP, ')']);
        xlabel('Time [s]');
        ylabel('Voltage [V]');
        legend(['V1'; 'V2']);
    end
end

%% ---- IMPORT ----

function data = import(name)
    % Define the filename
    filename = strcat('data/', name, '.txt'); % Input file path
    
    % Open the file
    file = fopen(filename, 'r');
    if file == -1
        error('Error opening file');
    end
    
    % Initialize data storage
    data = containers.Map;
    
    % Read file line by line
    while ~feof(file)
        line = strtrim(fgetl(file));
    
        % Detect step information line
        if startsWith(line, 'Step Information:')
            % Extract setup name
            setup_name = extractAfter(line, 'Step Information: ');
            setup_name = strtrim(setup_name);
            setup_name = extractBefore(setup_name, '  (Step:'); % Remove "(Step: X/Y)"
    
            % Extract Rref
            Rref_str = extractBetween(setup_name, 'Rref=', 'K F');
            if isempty(Rref_str)
                Rref = str2double(extractBetween(setup_name, 'Rref=', ' '));
            else
                Rref = str2double(Rref_str) * 1000; % Convert "k" to actual value
            end
    
    
            % Extract Freq
            Freq_str = extractBetween(setup_name, 'Freq=', 'K');
            if isempty(Freq_str)
                Freq = str2double(extractAfter(setup_name, 'Freq='));
            else
                Freq = str2double(Freq_str) * 1000; % Convert "k" to actual value
            end
    
            % Initialize storage for this step
            time = [];
            V1 = [];
            V2 = [];
    
            % Read data until next step or EOF
            while ~feof(file)
                line = strtrim(fgetl(file));
                if isempty(line) || startsWith(line, 'Step Information:')
                    fseek(file, -length(line)-2, 'cof'); % Rewind for next step
                    break;
                end
    
                % Read numerical values (ignore V(n001), V(n002), I(V3))
                values = sscanf(line, '%f %f %f');
                time(end+1) = values(1);
                V1(end+1) = values(2);
                V2(end+1) = values(3);
            end
    
            % Store extracted data
            step_data = struct();
            step_data.time = time;
            step_data.V1 = V1;
            step_data.V2 = V2;
            step_data.Rref = Rref;
            step_data.Freq = Freq;
    
            % Store in map with setup name as key
            data(setup_name) = step_data;
        end
    end
    
    % Close the file
    fclose(file);

end
