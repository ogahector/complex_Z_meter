import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# =============================================================================
# Define the model functions for the two equivalent circuits.
# =============================================================================

def series_model(freq, R, L, C):
    """
    Computes the complex impedance for a series RLC circuit.
    
    Z(ω) = R + j*(ωL - 1/(ωC))
    
    Parameters:
        freq : Frequency (Hz) or array of frequencies.
        R : Resistance in Ohm.
        L : Inductance in Henries.
        C : Capacitance in Farads.
        
    Returns:
        Complex impedance at the given frequency/frequencies.
    """
    omega = 2 * np.pi * freq
    return R + 1j * (omega * L - 1/(omega * C))

def parallel_model(freq, R, L, C):
    """
    Computes the complex impedance for a parallel RLC circuit.
    
    First, calculate the total admittance:
        Y(ω) = 1/R + j*(ωC - 1/(ωL))
    Then, the impedance is:
        Z(ω) = 1 / Y(ω)
    
    Parameters:
        freq : Frequency (Hz) or array of frequencies.
        R : Resistance in Ohm.
        L : Inductance in Henries.
        C : Capacitance in Farads.
        
    Returns:
        Complex impedance at the given frequency/frequencies.
    """
    omega = 2 * np.pi * freq
    Y = 1/R + 1j * (omega * C - 1/(omega * L))
    return 1 / Y

# For curve fitting, we need to work with real numbers.
# We define wrapper functions that return a concatenated array of the real and imaginary parts.
def series_model_fit(freq, R, L, C):
    Z = series_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

def parallel_model_fit(freq, R, L, C):
    Z = parallel_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

# =============================================================================
# Data preparation: convert measurement dictionary to arrays.
# =============================================================================

def prepare_data(measurement_dict):
    """
    Converts the measurement dictionary into frequency and complex impedance arrays.
    
    Parameters:
        measurement_dict : dict[int, complex]
            Dictionary with frequency (Hz) keys and complex impedance values.
            
    Returns:
        freqs : Sorted numpy array of frequencies.
        Z_measured : Numpy array of complex impedance values.
        data : Concatenated array of real and imaginary parts for fitting.
    """
    freqs = np.array(sorted(measurement_dict.keys()))
    Z_measured = np.array([measurement_dict[f] for f in freqs])
    # Concatenate the real and imaginary parts for use with curve_fit
    data = np.concatenate([np.real(Z_measured), np.imag(Z_measured)])
    return freqs, Z_measured, data

# =============================================================================
# Fit function for a given model.
# =============================================================================

def fit_model(model_func, freqs, data, initial_guess, bounds):
    """
    Fits the given model function to the measurement data using non-linear least squares.
    
    Parameters:
        model_func : function
            The model function that returns a concatenated real-imaginary array.
        freqs : Array of frequencies.
        data : Concatenated measured data (real and imaginary parts).
        initial_guess : List of initial guesses for [R, L, C].
        bounds : Tuple of lower and upper bounds for parameters.
        
    Returns:
        popt : Optimal values for the parameters.
        perr : Estimated standard deviations (error estimates) for the parameters.
        ssq : Sum of squared residuals.
    """
    # Use curve_fit to perform non-linear least squares fitting.
    popt, pcov = curve_fit(model_func, freqs, data, p0=initial_guess, bounds=bounds)
    # Compute residuals and sum-of-squared errors
    residuals = data - model_func(freqs, *popt)
    ssq = np.sum(residuals**2)
    # Standard deviation errors for the parameters are the square roots of the diagonal of the covariance matrix.
    perr = np.sqrt(np.diag(pcov))
    return popt, perr, ssq

# =============================================================================
# Main function: determine best model and plot the results.
# =============================================================================

def main(measurement_dict):
    # Prepare the measurement data.
    freqs, Z_measured, data = prepare_data(measurement_dict)

    # --- Initial Guesses ---
    # Use the median of the real parts as an initial guess for R.
    # L and C are guessed based on typical values; adjust if needed.
    R0 = np.median(np.real(Z_measured))
    L0 = 0  # initial guess in Henries
    C0 = 0  # initial guess in Farads
    initial_guess = [R0, L0, C0]
    
    # Set bounds to enforce positive parameters.
    bounds = (0, np.inf)

    # --- Fit the series model ---
    try:
        popt_series, perr_series, ssq_series = fit_model(series_model_fit, freqs, data, initial_guess, bounds)
    except Exception as e:
        print("Series fit failed:", e)
        popt_series, perr_series, ssq_series = None, None, np.inf

    # --- Fit the parallel model ---
    try:
        popt_parallel, perr_parallel, ssq_parallel = fit_model(parallel_model_fit, freqs, data, initial_guess, bounds)
    except Exception as e:
        print("Parallel fit failed:", e)
        popt_parallel, perr_parallel, ssq_parallel = None, None, np.inf

    # --- Select the best model based on the sum-of-squared errors ---
    if ssq_series < ssq_parallel:
        best_model = "series"
        best_popt = popt_series
        best_perr = perr_series
        best_ssq = ssq_series
        model_func = series_model
    else:
        best_model = "parallel"
        best_popt = popt_parallel
        best_perr = perr_parallel
        best_ssq = ssq_parallel
        model_func = parallel_model

    # --- Display the results ---
    print("Best equivalent circuit model:", best_model)
    print("Fitted parameters (with 1-sigma error estimates):")
    print(f"R = {best_popt[0]:.3e} ± {best_perr[0]:.3e} Ohm")
    print(f"L = {best_popt[1]:.3e} ± {best_perr[1]:.3e} H")
    print(f"C = {best_popt[2]:.3e} ± {best_perr[2]:.3e} F")
    print(f"Sum of squared residuals: {best_ssq:.3e}")

    # --- Plot the measured data and model fit ---
    # Use a denser frequency grid for the fitted model curve.
    freq_dense = np.linspace(min(freqs), max(freqs), 1000)
    Z_model_dense = model_func(freq_dense, *best_popt)

    plt.figure(figsize=(12, 6))

    # Plot real parts of impedance.
    plt.subplot(2, 1, 1)
    plt.plot(freqs, np.real(Z_measured), 'o', label='Measured (Real)')
    plt.plot(freq_dense, np.real(Z_model_dense), '-', label='Model (Real)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Real Impedance (Ohm)')
    plt.legend()
    plt.grid(True)

    # Plot imaginary parts of impedance.
    plt.subplot(2, 1, 2)
    plt.plot(freqs, np.imag(Z_measured), 'o', label='Measured (Imaginary)')
    plt.plot(freq_dense, np.imag(Z_model_dense), '-', label='Model (Imaginary)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Imaginary Impedance (Ohm)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# =============================================================================
# Example usage with synthetic data for testing.
# =============================================================================

if __name__ == "__main__":
    # For demonstration, we generate synthetic data for a series RLC circuit.
    # True parameters for the synthetic series RLC circuit:
    true_R = 1000      # Ohm
    true_L = 1e-3     # Henries
    true_C = 1e-6    # Farads

    # Create a range of frequencies (Hz) for the measurement.
    freqs = np.linspace(1e3, 1e6, 50).astype(int)
    measurement_dict = {}

    # Generate synthetic impedance data with a little added noise.
    for f in freqs:
        omega = 2 * np.pi * f
        Z = true_R + 1j * (omega * true_L - 1/(omega * true_C))
        # Add Gaussian noise (5% of true_R as a rough noise level).
        noise = (np.random.normal(scale=0.05 * true_R) +
                 1j * np.random.normal(scale=0.05 * true_R))
        measurement_dict[f] = Z + noise

    # Run the main analysis function.
    main(measurement_dict)
