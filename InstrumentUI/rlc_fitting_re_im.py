import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# =============================================================================
# Model Definitions: Compute complex impedance for series and parallel RLC.
# =============================================================================
def series_model(freq, R, L, C):
    # RLC all in series
    omega = 2 * np.pi * freq
    return R + 1j * (omega * L - 1/(omega * C))

def parallel_model(freq, R, L, C):
    # RLC all in parallel
    omega = 2 * np.pi * freq
    Y = 1/R + 1j * (omega * C - 1/(omega * L))
    return 1 / Y

def resistor_model(freq, R, L, C):
    # RL in series, with C in shunt
    omega = 2 * np.pi * freq
    Y = 1j * omega * C + 1 / (R + 1j * omega * L)
    return 1 / Y

def high_value_res_model(freq, R, L, C):
    omega = 2 * np.pi * freq
    return 1j * omega * L + 1 / (1j * omega * C + 1/R)

# =============================================================================
# Wrappers for Real and Imaginary Part Fitting
# =============================================================================
def series_model_fit(freq, R, L, C):
    Z = series_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

def parallel_model_fit(freq, R, L, C):
    Z = parallel_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

def resistor_model_fit(freq, R, L, C):
    Z = resistor_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

def high_value_res_model_fit(freq, R, L, C):
    Z = high_value_res_model(freq, R, L, C)
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
        ssq = np.sum(residuals**2)
        
        # Check convergence: if the change in parameters is below tolerance.
        if np.all(np.abs(popt - current_guess) < tol):
            print(f"Convergence reached after {iteration+1} iterations.")
            break
        
        current_guess = popt  # update the initial guess
    else:
        print("Maximum iterations reached without full convergence.")
    
    perr = np.sqrt(np.diag(pcov))
    return popt, perr, ssq

# =============================================================================
# Data Structure for Models
# =============================================================================
rlc_models = {
    "series": {
        "model": series_model,
        "model_fit": series_model_fit,
    },
    "parallel": {
        "model": parallel_model,
        "model_fit": parallel_model_fit,
    },
    "resistor": {
        "model": resistor_model,
        "model_fit": resistor_model_fit,
    },
    "highres": {
        "model": high_value_res_model,
        "model_fit": high_value_res_model_fit,
    }
}

# =============================================================================
# Main Analysis: Compare Models with Iterative Fitting.
# =============================================================================
def main(measurement_dict, freqlog=False):
    # Prepare measured data.
    freqs, Z_measured, data = prepare_data_real_imag(measurement_dict)
    
    # --- Initial Guess and Bounds ---
    R0 = np.median(np.real(Z_measured))
    L0 = 1e-9  # in Henries
    C0 = 1e-12  # in Farads
    initial_guess = [R0, L0, C0]
    bounds = (0, np.inf)  # Only positive values.
    
    # Dictionary to store results for each model.
    results = {}
    
    # Iterate over each model in the data structure.
    for name, funcs in rlc_models.items():
        try:
            popt, perr, ssq = iterative_fit(funcs["model_fit"], freqs, data, initial_guess, bounds)
        except Exception as e:
            print(f"{name} model iterative fit failed: {e}")
            popt, perr, ssq = None, None, np.inf
        results[name] = {"popt": popt, "perr": perr, "ssq": ssq, 
                          "model": funcs["model"], "model_fit": funcs["model_fit"]}
    
    # Select the best model based on SSQ.
    best_model_name = None
    best_ssq = np.inf
    for name, res in results.items():
        if res["ssq"] < best_ssq:
            best_ssq = res["ssq"]
            best_model_name = name
    
    if best_model_name is None:
        raise Exception("No valid model found.")
    
    best_res = results[best_model_name]
    best_popt = best_res["popt"]
    best_perr = best_res["perr"]
    best_model = best_res["model"]
    best_model_fit = best_res["model_fit"]
    
    # --- Display Results ---
    print("Best equivalent circuit model:", best_model_name)
    print("Fitted parameters (with 1-sigma error estimates):")
    print(f"R = {best_popt[0]:.3e} ± {best_perr[0]:.3e} Ohm")
    print(f"L = {best_popt[1]:.3e} ± {best_perr[1]:.3e} H")
    print(f"C = {best_popt[2]:.3e} ± {best_perr[2]:.3e} F")
    print(f"Sum of squared residuals: {best_res['ssq']:.3e}")
    
    # --- Plotting: Compare Measured Data with the Fitted Model ---
    freq_dense = np.linspace(min(freqs), max(freqs), 1000)
    Z_model_dense = best_model(freq_dense, *best_popt)
    
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    if freqlog:
        plt.semilogx(freqs, np.real(Z_measured), 'o', label='Measured (Real)')
        plt.semilogx(freq_dense, np.real(Z_model_dense), '-', label='Fitted Model (Real)')
    else:
        plt.plot(freqs, np.real(Z_measured), 'o', label='Measured (Real)')
        plt.plot(freq_dense, np.real(Z_model_dense), '-', label='Fitted Model (Real)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Real Impedance (Ohm)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    if freqlog:
        plt.semilogx(freqs, np.imag(Z_measured), 'o', label='Measured (Imag)')
        plt.semilogx(freq_dense, np.imag(Z_model_dense), '-', label='Fitted Model (Imag)')
    else:
        plt.plot(freqs, np.imag(Z_measured), 'o', label='Measured (Imag)')
        plt.plot(freq_dense, np.imag(Z_model_dense), '-', label='Fitted Model (Imag)')
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

    # # Generate frequencies from 1 kHz to 1 MHz.
    # freqs = np.linspace(1e2, 1e6, 50).astype(int)
    # measurement_dict = {}
    
    # # Generate synthetic impedance data with some Gaussian noise.
    # for f in freqs:
    #     omega = 2 * np.pi * f
    #     Z = true_R + 1j * (omega * true_L - 1/(omega * true_C))
    #     noise = np.abs(np.random.normal(scale=0.20))
    #     measurement_dict[f] = Z * (1 + noise)

    data = [100, 2508.7, 1.2144, 126, 1260.2, 6.1414, 158, 1181.9, 5.917, 200, 852.24, 0.21708, 251, 910.45, 6.17, 316, 1055.4, 6.0608, 398, 968.85, 6.113, 501, 932.18, 0.034196, 631, 927.37, 0.052376, 794, 966.38, 0.044535, 1000, 979.91, 0.02937, 1259, 1026.2, 0.0082968, 1587, 964.5, 6.2536, 2000, 1009.3, 6.262, 2512, 1004.5, 6.2329, 3164, 1000.7, 6.2095, 4000, 991.85, 6.1789, 5050, 1002.6, 6.1632, 6329, 998.1, 6.1355, 8064, 992.91, 6.0915, 10000, 1000.2, 6.0486, 12820, 990.97, 5.9847, 16129, 983.51, 5.9104, 20000, 981.18, 5.8179, 26315, 962.25, 5.6729, 33333, 943.69, 5.5179, 41666, 915.75, 5.3349, 55555, 863.16, 5.0423, 71428, 800.2, 4.7151, 83333, 752.93, 4.4887]

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
    print(measurement_dict)


    main(measurement_dict, freqlog=True)
