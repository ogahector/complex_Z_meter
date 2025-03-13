import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# =============================================================================
# Model Definitions: Compute complex impedance for series and parallel RLC.
# =============================================================================

def series_model(freq, R, L, C):
    """
    Series RLC circuit impedance:
      Z(ω) = R + j*(ωL - 1/(ωC))
    """
    omega = 2 * np.pi * freq
    return R + 1j * (omega * L - 1/(omega * C))

def parallel_model(freq, R, L, C):
    """
    Parallel RLC circuit impedance:
      Y(ω) = 1/R + j*(ωC - 1/(ωL))
      Z(ω) = 1 / Y(ω)
    """
    omega = 2 * np.pi * freq
    Y = 1/R + 1j * (omega * C - 1/(omega * L))
    return 1 / Y

# =============================================================================
# Wrappers for Real and Imaginary Part Fitting
# =============================================================================

def series_model_fit(freq, R, L, C):
    """
    For the series RLC model, returns a concatenated array of real and imaginary parts.
    """
    Z = series_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

def parallel_model_fit(freq, R, L, C):
    """
    For the parallel RLC model, returns a concatenated array of real and imaginary parts.
    """
    Z = parallel_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

# =============================================================================
# Data Preparation
# =============================================================================

def prepare_data_real_imag(measurement_dict):
    """
    Converts a dictionary of frequency (Hz) to complex impedance into arrays.
    Returns:
      - Sorted frequencies.
      - Array of measured complex impedances.
      - Concatenated real and imaginary parts for fitting.
    """
    freqs = np.array(sorted(measurement_dict.keys()))
    Z_measured = np.array([measurement_dict[f] for f in freqs])
    data = np.concatenate([np.real(Z_measured), np.imag(Z_measured)])
    return freqs, Z_measured, data

# =============================================================================
# Iterative Refinement for the Fitting Process
# =============================================================================

def iterative_fit(model_func, freqs, data, initial_guess, bounds, tol=1e-6, max_iter=10):
    """
    Iteratively refines the fit parameters.
    
    Parameters:
      model_func  : The model function returning concatenated real and imaginary parts.
      freqs       : Array of frequency values.
      data        : Measured data (concatenated real and imaginary parts).
      initial_guess: Initial guess for [R, L, C].
      bounds      : Bounds on the parameters.
      tol         : Convergence tolerance (change in parameters).
      max_iter    : Maximum number of iterations.
      
    Returns:
      popt        : Optimal parameters.
      perr        : Parameter uncertainties (1-sigma).
      ssq         : Sum of squared residuals.
    """
    current_guess = np.array(initial_guess)
    for iteration in range(max_iter):
        popt, pcov = curve_fit(model_func, freqs, data, p0=current_guess, bounds=bounds)
        residuals = data - model_func(freqs, *popt)
        ssq = np.mean(residuals**2)
        
        # Check convergence: if the change in parameters is below the tolerance.
        if np.all(np.abs(popt - current_guess) < tol):
            print(f"Convergence reached after {iteration+1} iterations.")
            break
        
        current_guess = popt  # Update the initial guess for the next iteration.
    else:
        print("Maximum iterations reached without full convergence.")
    
    perr = np.sqrt(np.diag(pcov))
    return popt, perr, ssq

# =============================================================================
# Main Analysis: Compare Series and Parallel Models with Iterative Fitting.
# =============================================================================

def main(measurement_dict):
    # Prepare measured data.
    freqs, Z_measured, data = prepare_data_real_imag(measurement_dict)
    
    # --- Initial Guess ---
    # Use the median of the real part as a rough estimate for R.
    R0 = np.median(np.real(Z_measured))
    L0 = 1e-5  # Initial guess for L (Henries)
    C0 = 1e-12  # Initial guess for C (Farads)
    initial_guess = [R0, L0, C0]
    bounds = (0, np.inf)  # Only positive values.
    
    # --- Iterative Fit for Series Model ---
    try:
        popt_series, perr_series, ssq_series = iterative_fit(series_model_fit, freqs, data,
                                                              initial_guess, bounds)
    except Exception as e:
        print("Series model iterative fit failed:", e)
        popt_series, perr_series, ssq_series = None, None, np.inf
    
    # --- Iterative Fit for Parallel Model ---
    try:
        popt_parallel, perr_parallel, ssq_parallel = iterative_fit(parallel_model_fit, freqs, data,
                                                                  initial_guess, bounds)
    except Exception as e:
        print("Parallel model iterative fit failed:", e)
        popt_parallel, perr_parallel, ssq_parallel = None, None, np.inf
    
    # --- Select the Best Model Based on Sum-of-Squared Residuals ---
    if ssq_series < ssq_parallel:
        best_model = "series"
        best_popt = popt_series
        best_perr = perr_series
        best_ssq = ssq_series
        model_func = series_model
        model_fit_func = series_model_fit
    else:
        best_model = "parallel"
        best_popt = popt_parallel
        best_perr = perr_parallel
        best_ssq = ssq_parallel
        model_func = parallel_model
        model_fit_func = parallel_model_fit
    
    # --- Display Results ---
    print("Best equivalent circuit model:", best_model)
    print("Fitted parameters (with 1-sigma error estimates):")
    print(f"R = {best_popt[0]:.3e} ± {best_perr[0]:.3e} Ohm")
    print(f"L = {best_popt[1]:.3e} ± {best_perr[1]:.3e} H")
    print(f"C = {best_popt[2]:.3e} ± {best_perr[2]:.3e} F")
    print(f"Sum of squared residuals: {best_ssq:.3e}")
    
    # --- Plotting: Compare Measured Data with the Fitted Model ---
    freq_dense = np.linspace(min(freqs), max(freqs), 1000)
    Z_model_dense = model_func(freq_dense, *best_popt)
    
    plt.figure(figsize=(12, 6))
    
    # Plot the Real parts.
    plt.subplot(2, 1, 1)
    plt.plot(freqs, np.real(Z_measured), 'o', label='Measured (Real)')
    plt.plot(freq_dense, np.real(Z_model_dense), '-', label='Fitted Model (Real)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Real Impedance (Ohm)')
    plt.legend()
    plt.grid(True)
    
    # Plot the Imaginary parts.
    plt.subplot(2, 1, 2)
    plt.plot(freqs, np.imag(Z_measured), 'o', label='Measured (Imaginary)')
    plt.plot(freq_dense, np.imag(Z_model_dense), '-', label='Fitted Model (Imaginary)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Imaginary Impedance (Ohm)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

# =============================================================================
# Example Usage with Synthetic Data for a Series RLC Circuit.
# =============================================================================

if __name__ == "__main__":
    true_R = 100      # Ohm
    true_L = 1e-3     # Henries
    true_C = 1e-6     # Farads

    # Generate frequencies from 1 kHz to 1 MHz.
    freqs = np.linspace(1e3, 1e6, 50).astype(int)
    measurement_dict = {}
    
    # Generate synthetic impedance data with some Gaussian noise.
    for f in freqs:
        omega = 2 * np.pi * f
        Z = true_R + 1j * (omega * true_L - 1/(omega * true_C))
        noise = (np.random.normal(scale=0.05 * true_R) +
                 1j * np.random.normal(scale=0.05 * true_R))
        measurement_dict[f] = Z + noise

    main(measurement_dict)
