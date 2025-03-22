import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# =============================================================================
# Model Definitions: All four circuit configurations
# =============================================================================
def series_model(freq, R, L, C):
    """Series RLC: Z = R + jωL + 1/(jωC)"""
    omega = 2 * np.pi * freq
    return R + 1j*omega*L + 1/(1j*omega*C)

def parallel_model(freq, R, L, C):
    """Parallel RLC: Y = 1/R + jωC + 1/(jωL)"""
    omega = 2 * np.pi * freq
    Y = 1/R + 1j*omega*C + 1/(1j*omega*L)
    return 1/Y

def resistor_model(freq, R, L, C):
    """RL series with C shunt: Z = 1/(jωC + 1/(R + jωL))"""
    omega = 2 * np.pi * freq
    return 1/(1j*omega*C + 1/(R + 1j*omega*L))

def high_value_res_model(freq, R, L, C):
    """L series with RC parallel: Z = jωL + 1/(1/R + jωC)"""
    omega = 2 * np.pi * freq
    return 1j*omega*L + 1/(1/R + 1j*omega*C)

# =============================================================================
# Magnitude/Phase Fitting Wrappers
# =============================================================================
def create_fit_wrapper(model_func):
    """Factory function to create magnitude/phase fitting wrappers"""
    def wrapper(freq, R, L, C):
        Z = model_func(freq, R, L, C)
        mag = np.abs(Z)
        phase = np.angle(Z)
        return np.concatenate([mag, phase])
    return wrapper

series_fit = create_fit_wrapper(series_model)
parallel_fit = create_fit_wrapper(parallel_model)
resistor_fit = create_fit_wrapper(resistor_model)
highres_fit = create_fit_wrapper(high_value_res_model)

# =============================================================================
# Data Preparation
# =============================================================================
def prepare_data(measurements):
    """Convert measurement dict to sorted arrays with magnitude/phase"""
    freqs = np.array([ measurements[i] for i in range(len(measurements)) if i % 3 == 0 ])

    mags = np.array([ measurements[i] for i in range(len(measurements)) if i % 3 == 1 ])
    phases = np.array([ measurements[i] for i in range(len(measurements)) if i % 3 == 2 ])

    Z_meas = mags * np.exp(1j * phases)

    mag = np.abs(Z_meas)
    phase = np.angle(Z_meas)
    return freqs, Z_meas, np.concatenate([mag, phase])

# =============================================================================
# Iterative Fitting Engine
# =============================================================================
def iterative_fit(model_func, freqs, data, init_guess, bounds, tol=1e-9, max_iter=10, method="trf"):
    """Robust fitting with parameter convergence checking"""
    current_guess = np.array(init_guess)
    for iter in range(max_iter):
        try:
            popt, pcov = curve_fit(model_func, freqs, data, 
                                  p0=current_guess, bounds=bounds, method=method,
                                  max_nfev=10000)
            residuals = data - model_func(freqs, *popt)
            ssq = np.sum(residuals**2)
            
            if np.all(np.abs(popt - current_guess) < tol):
                print(f"Converged after {iter+1} iterations")
                break
                
            current_guess = popt
        except Exception as e:
            print(f"Fit failed at iteration {iter+1}: {str(e)}")
            return None, None, np.inf
            
    else:
        print("Max iterations reached without convergence")
        
    try:  # Handle potential singular covariance matrix
        perr = np.sqrt(np.diag(pcov))
    except:
        perr = np.full_like(popt, np.nan)
        
    return popt, perr, ssq

# =============================================================================
# Model Configuration Database
# =============================================================================
models = {
    'series': {
        'function': series_model,
        'fit_wrapper': series_fit,
        'bounds': (0, np.inf)
    },
    'parallel': {
        'function': parallel_model,
        'fit_wrapper': parallel_fit,
        'bounds': (0, np.inf)
    },
    'resistor': {
        'function': resistor_model,
        'fit_wrapper': resistor_fit,
        'bounds': (0, np.inf)
    },
    'highres': {
        'function': high_value_res_model,
        'fit_wrapper': highres_fit,
        'bounds': (0, np.inf)
    }
}

# =============================================================================
# Main Analysis Routine
# =============================================================================
def analyze_impedance(measurements):
    """Main analysis workflow: fitting, comparison, and visualization"""
    freqs, Z_meas, fit_data = prepare_data(measurements)
    
    # Initial parameter estimation
    R_est = np.median(np.real(Z_meas))
    L_est = 1e-6  # Conservative guess for typical inductances
    C_est = 1e-9  # Conservative guess for typical capacitances
    init_params = [R_est, L_est, C_est]
    
    results = {}
    for name, model in models.items():
        params, errors, ssq = iterative_fit(
            model['fit_wrapper'], freqs, fit_data,
            init_params, model['bounds']
        )
        results[name] = {
            'params': params,
            'errors': errors,
            'ssq': ssq,
            'model': model['function']
        }
    
    # Model selection by sum of squares
    best_name = min(results, key=lambda k: results[k]['ssq'])
    best = results[best_name]
    print(f"\nBest model: {best_name} (SSQ = {best['ssq']:.2e})")
    
    # Parameter reporting
    print("\nFitted parameters with uncertainties:")
    for p, (val, err) in enumerate(zip(best['params'], best['errors'])):
        print(f"{['R','L','C'][p]} = {val:.3e} ± {err:.3e}")
    
    # Generate model predictions
    f_plot = np.geomspace(min(freqs), max(freqs), 500)
    Z_best = best['model'](f_plot, *best['params'])
    
    # Create diagnostic plots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Magnitude plot
    ax1.semilogx(freqs, np.abs(Z_meas), 'o', label='Measured')
    ax1.semilogx(f_plot, np.abs(Z_best), '-', label='Model')
    ax1.set_title(f'Best Fit: {best_name} Model')
    ax1.set_ylabel('|Z| (Ω)')
    ax1.legend()
    ax1.grid(True, which='both', linestyle='--')
    
    # Phase plot
    ax2.semilogx(freqs, np.unwrap(np.angle(Z_meas)), 'o', label='Measured')
    ax2.semilogx(f_plot, np.unwrap(np.angle(Z_best)), '-', label='Model')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Phase (rad)')
    ax2.legend()
    ax2.grid(True, which='both', linestyle='--')
    
    plt.tight_layout()
    plt.show()
    
    return best_name, best

def main():
    # Synthetic example: Series RLC
    true_R = 50
    true_L = 1e-4
    true_C = 1e-9

    freqs = np.logspace(2, 6, 60)  # from 1 kHz to 1 MHz
    data_list = []
    for f in freqs:
        Z = series_model(f, true_R, true_L, true_C)
        # Add some noise
        Z_noisy = Z * (1 + 0.01*np.random.randn()) + 1j*(0.01*np.random.randn())
        mag = np.abs(Z_noisy)
        phase = np.angle(Z_noisy)
        data_list.extend([f, mag, phase])

    data = [100, 96.501, -3.0725, 126, 450.28, -3.0176, 158, 15.93, -0.71009, 200, 65.354, -2.5703, 251, 41.311, -0.84842, 316, 7.9516, 0.79363, 398, 9.0305, 0.10746, 501, 10.462, 0.55468, 631, 4.4881, -0.77958, 794, 8.4864, 0.21163, 1000, 14.002, 0.17652, 1259, 6.5227, -0.39002, 1587, 12.668, 0.18917, 2000, 10.071, -0.024883, 2512, 9.7974, 0.019536, 3164, 11.832, 0.35374, 4000, 9.7971, 0.010781, 5050, 10.057, -0.079714, 6329, 10.15, -0.072141, 8064, 10.187, -0.069057, 10000, 9.8224, 0.067995, 12820, 10.14, -0.0020733, 16129, 10.124, 0.004766, 20000, 8.6804, -0.063401, 26315, 9.9627, 0.008273, 33333, 5.6324, -0.23971, 41666, 10.049, 0.0098545, 55555, 10.256, 0.014278, 71428, 8.3908, -0.0311, 83333, 7.984, -0.051899]
    data10nF = [100, 65910.0, 3.7857, 126, 34584.0, 1.5194, 158, 201290.0, 1.5796, 200, 88228.0, 1.2482, 251, 58805.0, 1.8933, 316, 45507.0, 1.4742, 398, 43590.0, 1.5965, 501, 32343.0, 1.6862, 631, 27280.0, 1.5771, 794, 21915.0, 1.5811, 1000, 16628.0, 1.5833, 1259, 13473.0, 1.6077, 1587, 10860.0, 1.5543, 2000, 8253.0, 1.5837, 2512, 6753.5, 1.589, 3164, 5255.2, 1.5729, 4000, 4234.7, 1.5851, 5050, 3303.3, 1.5578, 6329, 2655.0, 1.5575, 8064, 2095.7, 1.5374, 10000, 1620.6, 1.5154, 12820, 1318.1, 1.5482, 16129, 1043.4, 1.5242, 20000, 954.93, 1.6984, 26315, 629.95, 1.5067, 33333, 376.08, 1.4983, 41666, 403.98, 1.422, 55555, 306.45, 1.4167, 71428, 232.99, 1.1215, 83333, 207.51, 1.4859]

    data = data10nF[6:]

    # Fit
    # rlc_fit_mag_phase(data, freqlog=True)
    analyze_impedance(data)


if __name__ == "__main__":
    main()
