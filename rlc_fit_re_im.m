function [best_model_name, results] = rlc_fit_re_im(frequency, magnitude, phase, L0, C0)
    % Default parameter values if not provided
    if nargin < 4  % If L0 is not provided, set it to a reasonable default (e.g., 0)
        L0 = 0;
    end
    if nargin < 5  % If C0 is not provided, set it to a reasonable default (e.g., 0)
        C0 = 0;
    end

    % Define the rlc_models structure with model, model_fit, and bounds
    rlc_models = struct();
    
    % Series model (RLC all in series)
    series_model = @(params, freq) (params(1) + 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3)));
    series_model_fit = @(params, freq) series_model(params, freq);
    rlc_models.series.model = series_model;
    rlc_models.series.model_fit = series_model_fit;

    % Resistor model (RL in series, with C in shunt)
    resistor_model = @(params, freq) 1 ./ (1i .* 2 .* pi .* freq .* params(3) + 1 ./ (params(1) + 1i .* 2 .* pi .* freq .* params(2)));
    resistor_model_fit = @(params, freq) resistor_model(params, freq);
    rlc_models.resistor.model = resistor_model;
    rlc_models.resistor.model_fit = resistor_model_fit;

    % High value resistor model
    high_value_res_model = @(params, freq) 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3) + 1 ./ params(1));
    high_value_res_model_fit = @(params, freq) high_value_res_model(params, freq);
    rlc_models.highres.model = high_value_res_model;
    rlc_models.highres.model_fit = high_value_res_model_fit;

    % Parallel model (RLC all in parallel)
    parallel_model = @(params, freq) 1 ./ (params(1) + 1i .* 2 .* pi .* freq .* params(2) + 1 ./ (1i .* 2 .* pi .* freq .* params(3)));
    parallel_model_fit = @(params, freq) parallel_model(params, freq);
    rlc_models.parallel.model = parallel_model;
    rlc_models.parallel.model_fit = parallel_model_fit;

    % Prepare the complex impedance data from frequency, magnitude, and phase
    Z_measured = magnitude .* exp(1i .* phase);  % Convert magnitude and phase to complex impedance
    
    % Filter out data where frequency is below 300 Hz
    freq_mask = frequency >= 300;  % Logical mask for frequencies >= 300 Hz
    frequency = frequency(freq_mask);
    Z_measured = Z_measured(freq_mask);
    
    % Data for real fitting
    data = Z_measured;  % Concatenating real and imaginary parts into a single vector
    data_conj = conj(Z_measured);  % For conjugated fitting
    
    % --- Initial Guess and Bounds ---
    R0 = median(abs(real(Z_measured)));  % Initial guess for resistance (based on the median real part)
    % Use L0 and C0 passed as optional arguments
    initial_guess = [R0, L0, C0];
    
    % Dictionary to store results for each model
    results = containers.Map;
    
    % Iterate over each model in the data structure
    model_names = fieldnames(rlc_models);  % Use fieldnames instead of keys
    
    for i = 1:numel(model_names)
        name = model_names{i};  % Extract model name
        try
            funcs = rlc_models.(name);  % Access the model functions
            
            % Use fmincon to fit the data (complex impedance model)
            options = optimoptions('fmincon', ...
                                    'Display', 'off', ...  % No display output
                                    'TolFun', 1e-15, ...    % Tolerance for the objective function value
                                    'TolX', 1e-15, ...      % Tolerance for the parameter values
                                    'TolCon', 1e-15, ...    % Tolerance for the constraints
                                    'MaxFunEvals', 1e9, ... % Maximum number of function evaluations
                                    'MaxIter', 1e4, ...     % Maximum number of iterations
                                    'StepTolerance', 1e-15, ...  % Step tolerance
                                    'OptimalityTolerance', 1e-15); % Tolerance for optimality
            
            % Define the objective function (sum of squared residuals)
            objective = @(params) sum(abs(data - funcs.model(params, frequency)).^2);
            
            % Bounds (lower and upper bounds for R, L, and C)
            lb = [0, 0.9e-3, 0.9e-9];  % Lower bounds for R, L, and C
            % lb = [0, 0, 0];
            ub = [Inf, 1.1e-1, 1.1e-6];  % Upper bounds for R, L, and C
            % ub = [Inf, 1, 1];
            
            % Call fmincon to perform the optimization
            [popt, fval] = fmincon(objective, initial_guess, [], [], [], [], lb, ub, [], options);
            
            % After fitting, calculate the error (standard error using residuals)
            residuals = data - funcs.model(popt, frequency);
            ssq = sum(abs(residuals).^2);
            
            % Store the results in the dictionary
            results(name) = struct('popt', popt, 'ssq', ssq, 'model', funcs.model, 'model_fit', funcs.model_fit);

        catch e
            fprintf('%s model iterative fit failed: %s\n', name, e.message);
            popt = NaN; ssq = Inf;
        end
    end
    
    % Select the best model based on SSQ
    best_model_name = '';
    best_ssq = Inf;
    for i = 1:numel(model_names)
        name = model_names{i};  % Extract model name
        res = results(name);
        if res.ssq < best_ssq
            best_ssq = res.ssq;
            best_model_name = name;
        end
    end
    
    if isempty(best_model_name)
        error('No valid model found.');
    end
    
    best_res = results(best_model_name);
    best_popt = best_res.popt;
    
    % Display results
    fprintf('Best model: %s\n', best_model_name);
    % fprintf('Fitted parameters:\n');
    % fprintf('R = %.3e Ohm\n', best_popt(1));
    % fprintf('L = %.3e H\n', best_popt(2));
    % fprintf('C = %.3e F\n', best_popt(3));
    % fprintf('Sum of squared residuals: %.3e\n', best_res.ssq);
end
