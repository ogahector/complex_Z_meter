import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# =============================================================================
# Model definitions for series and parallel RLC circuits.
# =============================================================================

def series_model(freq, R, L, C):
    """Series RLC impedance: Z(ω) = R + j*(ωL - 1/(ωC))."""
    omega = 2 * np.pi * freq
    return R + 1j * (omega * L - 1/(omega * C))

def parallel_model(freq, R, L, C):
    """
    Parallel RLC impedance:
        Y(ω) = 1/R + j*(ωC - 1/(ωL))
        Z(ω) = 1/Y(ω)
    """
    omega = 2 * np.pi * freq
    Y = 1/R + 1j * (omega * C - 1/(omega * L))
    return 1 / Y

# =============================================================================
# Wrappers for fitting using magnitude and phase.
# =============================================================================

def series_model_fit_mag_phase(freq, R, L, C):
    """Wrapper for series model: returns concatenated magnitude and unwrapped phase."""
    Z = series_model(freq, R, L, C)
    mag = np.abs(Z)
    phase = np.unwrap(np.angle(Z))
    return np.concatenate([mag, phase])

def parallel_model_fit_mag_phase(freq, R, L, C):
    """Wrapper for parallel model: returns concatenated magnitude and unwrapped phase."""
    Z = parallel_model(freq, R, L, C)
    mag = np.abs(Z)
    phase = np.unwrap(np.angle(Z))
    return np.concatenate([mag, phase])

# =============================================================================
# Data preparation.
# =============================================================================

def prepare_data_mag_phase(measurement_dict):
    """
    Converts the measurement dictionary into frequency, measured impedance,
    and concatenated magnitude and phase arrays.
    """
    freqs = np.array(sorted(measurement_dict.keys()))
    Z_measured = np.array([measurement_dict[f] for f in freqs])
    measured_mag = np.abs(Z_measured)
    measured_phase = np.unwrap(np.angle(Z_measured))
    data = np.concatenate([measured_mag, measured_phase])
    return freqs, Z_measured, data

# =============================================================================
# Fitting routine with iterative refinement.
# =============================================================================

def iterative_fit(model_func, freqs, data, initial_guess, bounds, tol=1e-6, max_iter=10):
    """
    Iteratively refines the initial guess for the fit parameters.
    
    Parameters:
        model_func : Function returning concatenated magnitude and phase.
        freqs : Array of frequency values.
        data : Measured data (concatenated magnitude and phase).
        initial_guess : List or array with initial [R, L, C] guess.
        bounds : Tuple defining lower and upper bounds for parameters.
        tol : Tolerance for convergence (change in parameters).
        max_iter : Maximum number of iterations.
        
    Returns:
        popt : Optimal parameters after iterative refinement.
        perr : Parameter error estimates (1-sigma).
        ssq : Sum of squared residuals.
    """
    current_guess = np.array(initial_guess)
    for iteration in range(max_iter):
        popt, pcov = curve_fit(model_func, freqs, data, p0=current_guess, bounds=bounds)
        residuals = data - model_func(freqs, *popt)
        ssq = np.sum(residuals**2)
        
        # Check convergence: if change in parameters is below tolerance, break.
        if np.all(np.abs(popt - current_guess) < tol):
            print(f"Convergence reached after {iteration+1} iterations.")
            break
        
        current_guess = popt  # update the guess for the next iteration
    else:
        print("Maximum iterations reached without full convergence.")
    
    perr = np.sqrt(np.diag(pcov))
    return popt, perr, ssq

# =============================================================================
# Main analysis: Compare series vs. parallel models using iterative refinement.
# =============================================================================

def main(measurement_dict):
    # Prepare measured data (magnitude and phase).
    freqs, Z_measured, data = prepare_data_mag_phase(measurement_dict)

    # --- Initial Guesses ---
    # Use the median magnitude as an estimate for R.
    R0 = np.median(np.real(Z_measured))
    L0 = 1e-6  # Initial guess for inductance (H)
    C0 = 1e-9  # Initial guess for capacitance (F)
    initial_guess = [R0, L0, C0]
    bounds = (0, np.inf)  # Enforce positive values.

    # --- Iterative fit for the series RLC model ---
    try:
        popt_series, perr_series, ssq_series = iterative_fit(series_model_fit_mag_phase, freqs, data,
                                                               initial_guess, bounds)
    except Exception as e:
        print("Series model iterative fit failed:", e)
        popt_series, perr_series, ssq_series = None, None, np.inf

    # --- Iterative fit for the parallel RLC model ---
    try:
        popt_parallel, perr_parallel, ssq_parallel = iterative_fit(parallel_model_fit_mag_phase, freqs, data,
                                                                   initial_guess, bounds)
    except Exception as e:
        print("Parallel model iterative fit failed:", e)
        popt_parallel, perr_parallel, ssq_parallel = None, None, np.inf

    # --- Model selection based on sum-of-squared residuals ---
    if ssq_series < ssq_parallel:
        best_model = "series"
        best_popt = popt_series
        best_perr = perr_series
        best_ssq = ssq_series
        model_func = series_model
        model_fit_func = series_model_fit_mag_phase
    else:
        best_model = "parallel"
        best_popt = popt_parallel
        best_perr = perr_parallel
        best_ssq = ssq_parallel
        model_func = parallel_model
        model_fit_func = parallel_model_fit_mag_phase

    # --- Display fitted parameters ---
    print("Best equivalent circuit model:", best_model)
    print("Fitted parameters (with 1-sigma error estimates):")
    print(f"R = {best_popt[0]:.3e} ± {best_perr[0]:.3e} Ohm")
    print(f"L = {best_popt[1]:.3e} ± {best_perr[1]:.3e} H")
    print(f"C = {best_popt[2]:.3e} ± {best_perr[2]:.3e} F")
    print(f"Sum of squared residuals: {best_ssq:.3e}")

    # --- Plot measured vs. fitted data (magnitude and phase) ---
    freq_dense = np.linspace(min(freqs), max(freqs), 1000)
    Z_model_dense = model_func(freq_dense, *best_popt)
    model_mag_dense = np.abs(Z_model_dense)
    model_phase_dense = np.unwrap(np.angle(Z_model_dense))

    measured_mag = np.abs(Z_measured)
    measured_phase = np.unwrap(np.angle(Z_measured))

    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(freqs, measured_mag, 'o', label='Measured Magnitude')
    plt.plot(freq_dense, model_mag_dense, '-', label='Fitted Model Magnitude')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude (Ohm)')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(freqs, measured_phase, 'o', label='Measured Phase')
    plt.plot(freq_dense, model_phase_dense, '-', label='Fitted Model Phase')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Phase (radians)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# =============================================================================
# Example usage with synthetic data for demonstration.
# =============================================================================

if __name__ == "__main__":
    # Generate synthetic data for a series RLC circuit.
    true_R = 100      # Ohm
    true_L = 1e-3     # Henries
    true_C = 1e-9     # Farads

    freqs = np.linspace(1e3, 1e6, 50).astype(int)
    measurement_dict = {}

    for f in freqs:
        omega = 2 * np.pi * f
        Z = true_R + 1j * (omega * true_L - 1/(omega * true_C))
        noise = (np.random.normal(scale=0.05 * true_R) +
                 1j * np.random.normal(scale=0.05 * true_R))
        measurement_dict[f] = Z + noise

    data = [100, 3830.3, 0.63425, 126, 3145.5, 5.1053, 158, 1159.6, 5.8947, 200, 2097.0, 5.9274, 251, 866.14, 0.039018, 316, 1025.4, 0.25573, 398, 952.42, 6.1027, 501, 1078.1, 6.2502, 631, 1144.6, 6.1772, 794, 1147.0, 0.020207, 1000, 1019.0, 6.2234, 1259, 1076.6, 6.2507, 1587, 1041.7, 0.051835, 2000, 954.88, 0.031093, 2512, 970.64, 0.043148, 3164, 967.85, 0.027785, 4000, 1012.4, 0.031298, 5050, 976.94, 0.0086845, 6329, 1002.3, 0.010878, 8064, 999.9, 0.0088616, 10000, 1002.3, 0.00682, 12820, 1002.1, 0.021743, 16129, 1000.0, 0.033297, 20000, 995.4, 0.035552, 26315, 1001.3, 0.042906, 33333, 996.59, 0.057959, 41666, 1000.2, 0.072334, 55555, 997.77, 0.096274, 71428, 998.42, 0.11875, 83333, 1002.4, 0.13403]

    freqs = []
    mags = []
    phases = []
    for i in range(0, len(data)):
        if i % 3 == 0:
            freqs.append(data[i])
        elif i % 3 == 1:
            mags.append(data[i])
        elif i % 3 == 2:
            phases.append(data[i])

    measurement_dict = {freqs[i]: mags[i] * np.exp(1j * phases[i]) for i in range(len(freqs))}
    # Run the analysis.
    main(measurement_dict)
