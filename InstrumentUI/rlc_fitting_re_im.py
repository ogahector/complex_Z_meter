import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# =============================================================================
# Model Definitions: Compute complex impedance for series and parallel RLC.
# =============================================================================
def series_model(freq, R, L, C):
    # RLC all in series
    omega = 2 * np.pi * freq
    return R + 1j * omega * L + 1/(1j * omega * C)

def parallel_model(freq, R, L, C):
    # RLC all in parallel
    omega = 2 * np.pi * freq
    Y = 1/R + 1j * omega * C + 1/(1j * omega * L)
    return 1 / Y

def resistor_model(freq, R, L, C):
    # RL in series, with C in shunt
    omega = 2 * np.pi * freq
    # Y = 1j * omega * C + 1 / (R + 1j * omega * L)
    # return 1 / Y
    return 1 / ( 1j * omega * C + 1 / ( R + 1j * omega * L ) )

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
def prepare_data_real_imag(data):
    """
    Converts a flat array of frequency, magnitude and phase into arrays.
    Returns:
      - Sorted frequencies.
      - Array of measured complex impedances.
      - Concatenated real and imaginary parts for fitting.
    """
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

    freqs = np.array(sorted(freqs))

    freqbegin = 0   
    for freqbegin in range(len(freqs)):
        if freqs[freqbegin] > 600:
            break
    
    freqend = -1
    # for freqend in range(len(freqs)):
    #     if freqs[freqend] > 50e3:
    #         break

    freqs = freqs[freqbegin : freqend]
    mags = mags[freqbegin : freqend]
    phases = phases[freqbegin : freqend]

    Z_measured = np.array(mags * np.exp(  1j * np.array(phases) ))
    usable_data = np.concatenate([np.real(Z_measured), np.imag(Z_measured)])
    usable_data_conj = np.concatenate([np.real(Z_measured), - np.imag(Z_measured)])
    return freqs, Z_measured, usable_data, usable_data_conj

# =============================================================================
# Iterative Refinement for the Fitting Process
# =============================================================================
def iterative_fit(model_func, freqs, data, initial_guess, bounds=None, 
                  Rtol=0.1, Ltol=1e-9, Ctol=1e-12, 
                  max_iter=50, method="lm"):
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
    tol = 1e-10
    if method.lower() == "lm":
        bounds = None
    for iteration in range(max_iter):
        popt, pcov = curve_fit(model_func, freqs, data, 
                               p0=current_guess, bounds=bounds,
                               method=method)
        residuals = data - model_func(freqs, *popt)
        ssq = np.sum(residuals**2)
        
        # Check convergence: if the change in parameters is below tolerance.
        # if np.all(np.abs(popt - current_guess) < tol):
        if within_tolerance(*popt, *current_guess, Rtol, Ltol, Ctol):
            print(f"Convergence reached after {iteration+1} iterations.")
            break
        
        current_guess = popt  # update the initial guess
    else:
        print("Maximum iterations reached without full convergence.")
    
    perr = np.sqrt(np.diag(pcov))
    return popt, perr, ssq

def within_tolerance(R1, L1, C1, R2, L2, C2, Rtol, Ltol, Ctol):
    if(np.abs(R1 - R2) < Rtol):
        return False
    if(np.abs(L1 - L2) < Ltol):
        return False
    if(np.abs(C1 - C2) < Ctol):
        return False
    return True

# =============================================================================
# Data Structure for Models
# =============================================================================
rlc_models = {
    "series": {
        "model": series_model,
        "model_fit": series_model_fit,
        "bounds": ([0, 0, 0], [np.inf, 1, np.inf])  # Example bounds
    },
    "parallel": {
        "model": parallel_model,
        "model_fit": parallel_model_fit,
        "bounds": ([0, 0, 0], [np.inf, 1, 1])  # Example bounds
    },
    "resistor": {
        "model": resistor_model,
        "model_fit": resistor_model_fit,
        "bounds": ([0, 0, 0], [np.inf, 1, np.inf])  # Example bounds
    },
    "highres": {
        "model": high_value_res_model,
        "model_fit": high_value_res_model_fit,
        "bounds": ([0, 0, 0], [np.inf, 1, np.inf])  # Example bounds
    }
}

# =============================================================================
# Main Analysis: Compare Models with Iterative Fitting.
# =============================================================================
def rlc_fit_re_im(measurements, Rtol=0.1, Ltol=1e-9, Ctol=1e-12, plot_all=False) -> tuple[ str, dict ]:
    # Prepare measured data.
    freqs, Z_measured, data, data_conj = prepare_data_real_imag(measurements)
    
    # --- Initial Guess and Bounds ---
    R0 = np.median(np.abs(np.real(Z_measured)))
    # R0 = np.sqrt( 10 * 1e6 ) # gemoetric mean of impedances
    L0 = 1e-9  # in Henries
    C0 = 1e-12  # in Farads

    initial_guess = [R0, L0, C0]
    
    # Dictionary to store results for each model.
    results = {}
    
    # Iterate over each model in the data structure.
    for name, funcs in rlc_models.items():
        try:
            bounds = funcs["bounds"]
            popt, perr, ssq = iterative_fit(funcs["model_fit"], freqs, data, 
                                            initial_guess, bounds=bounds, 
                                            Rtol=Rtol, Ltol=Ltol, Ctol=Ctol, 
                                            method="trf")
            
            # popt_conj, perr_conj, ssq_conj = iterative_fit(funcs["model_fit"], freqs, data_conj,
            #                                                initial_guess, bounds=bounds,
            #                                                Rtol=Rtol, Ltol=Ltol, Ctol=Ctol,
            #                                                method="trf")
            
            # if ssq > ssq_conj:
            #     popt = popt_conj
            #     perr = perr_conj
            #     ssq = ssq_conj

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
    
    ## CHECK FOR CONJUGATE:
    # if best_model_name is None:
    #     print(results)
    #     # raise Exception("No valid model found.")
    #     print("Trying again but with inverted phase")

    #     for i in range(len(measurements)):
    #         if i % 3 == 2:
    #             measurements[i] = - measurements[i]

    #     freqs, Z_measured, data = prepare_data_real_imag(measurements)

    #     for name, funcs in rlc_models.items():
    #         try:
    #             bounds = funcs["bounds"]
    #             popt, perr, ssq = iterative_fit(funcs["model_fit"], freqs, data, 
    #                                             initial_guess, bounds=bounds, 
    #                                             Rtol=Rtol, Ltol=Ltol, Ctol=Ctol, 
    #                                             method="trf")

    #         except Exception as e:
    #             print(f"{name} model iterative fit failed: {e}")
    #             popt, perr, ssq = None, None, np.inf
    #         results[name] = {"popt": popt, "perr": perr, "ssq": ssq, 
    #                         "model": funcs["model"], "model_fit": funcs["model_fit"]}
            
    best_model_name = None
    best_ssq = np.inf
    for name, res in results.items():
        if res["ssq"] < best_ssq:
            best_ssq = res["ssq"]
            best_model_name = name

    if best_model_name is None:
        print(results)
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

    if plot_all:
        for name, res in results.items():
            popt = res['popt']
            model = res['model']

            if popt is None:
                print(f'Could not display model {name}')
                continue
            print(f'Displaying model {name}...')

            freq_dense = np.logspace(np.log10(freqs[0]), np.log10(freqs[-1]), 1000)
            Z_model_dense = model(freq_dense, *popt)

            f = plt.figure(figsize=(12, 6))
            plt.subplot(2, 1, 1)

            plt.semilogx(freqs, np.real(Z_measured), 'o', label='Measured (Real)')
            plt.semilogx(freq_dense, np.real(Z_model_dense), '-', label='Fitted Model (Real)')

            plt.xlabel('Frequency (Hz)')
            plt.ylabel('Real Impedance (Ohm)')
            plt.legend()
            plt.grid(True)

            plt.subplot(2, 1, 2)

            plt.semilogx(freqs, np.imag(Z_measured), 'o', label='Measured (Imag)')
            plt.semilogx(freq_dense, np.imag(Z_model_dense), '-', label='Fitted Model (Imag)')
            plt.xlabel('Frequency (Hz)')
            plt.ylabel('Imaginary Impedance (Ohm)')
            plt.legend()
            plt.grid(True)

            plt.title(f'Model {name}, with R: {popt[0]:.2e}, L: {popt[1]:.2e}, C: {popt[2]:.2e}')
            
            plt.tight_layout()

            f.show()
        
            plt.show()

    return best_model_name, best_res


def main(measurements, freqlog=False):
    plot_all = False
    best_model_name, best_res = rlc_fit_re_im(measurements, plot_all=plot_all)
    
    best_popt = best_res["popt"]
    best_perr = best_res["perr"]
    best_model = best_res["model"]
    best_model_fit = best_res["model_fit"]

    freqs, Z_measured, _, _ = prepare_data_real_imag(measurements)
    
    # --- Plotting: Compare Measured Data with the Fitted Model ---
    freq_dense = np.linspace(min(freqs), max(freqs), 1000)
    Z_model_dense = best_model(freq_dense, *best_popt)

    if plot_all: 
        return
    
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
    true_R = 1      # Ohm
    true_L = 1e-6     # Henries
    true_C = 1e-8     # Farads

    # Generate frequencies from 1 kHz to 1 MHz.
    freqs = np.logspace(2, 6, 50).astype(int)

    data_sim = []
    
    # Generate synthetic impedance data with some Gaussian noise.
    for f in freqs:
        omega = 2 * np.pi * f
        Z = true_R + 1j * (omega * true_L - 1/(omega * true_C))
        noise = np.abs(np.random.normal(scale=0.20))
        Z_noisy = Z * noise
        data_sim.extend([f, np.abs(Z_noisy), np.angle(Z_noisy)])
        

    data = [100, 3830.3, 0.63425, 126, 3145.5, 5.1053, 158, 1159.6, 5.8947, 200, 2097.0, 5.9274, 251, 866.14, 0.039018, 316, 1025.4, 0.25573, 398, 952.42, 6.1027, 501, 1078.1, 6.2502, 631, 1144.6, 6.1772, 794, 1147.0, 0.020207, 1000, 1019.0, 6.2234, 1259, 1076.6, 6.2507, 1587, 1041.7, 0.051835, 2000, 954.88, 0.031093, 2512, 970.64, 0.043148, 3164, 967.85, 0.027785, 4000, 1012.4, 0.031298, 5050, 976.94, 0.0086845, 6329, 1002.3, 0.010878, 8064, 999.9, 0.0088616, 10000, 1002.3, 0.00682, 12820, 1002.1, 0.021743, 16129, 1000.0, 0.033297, 20000, 995.4, 0.035552, 26315, 1001.3, 0.042906, 33333, 996.59, 0.057959, 41666, 1000.2, 0.072334, 55555, 997.77, 0.096274, 71428, 998.42, 0.11875, 83333, 1002.4, 0.13403]

    data10Ohms = [100, 96.501, -3.0725, 126, 450.28, -3.0176, 158, 15.93, -0.71009, 200, 65.354, -2.5703, 251, 41.311, -0.84842, 316, 7.9516, 0.79363, 398, 9.0305, 0.10746, 501, 10.462, 0.55468, 631, 4.4881, -0.77958, 794, 8.4864, 0.21163, 1000, 14.002, 0.17652, 1259, 6.5227, -0.39002, 1587, 12.668, 0.18917, 2000, 10.071, -0.024883, 2512, 9.7974, 0.019536, 3164, 11.832, 0.35374, 4000, 9.7971, 0.010781, 5050, 10.057, -0.079714, 6329, 10.15, -0.072141, 8064, 10.187, -0.069057, 10000, 9.8224, 0.067995, 12820, 10.14, -0.0020733, 16129, 10.124, 0.004766, 20000, 8.6804, -0.063401, 26315, 9.9627, 0.008273, 33333, 5.6324, -0.23971, 41666, 10.049, 0.0098545, 55555, 10.256, 0.014278, 71428, 8.3908, -0.0311, 83333, 7.984, -0.051899]
    data10nF = [100, 65910.0, 3.7857, 126, 34584.0, 1.5194, 158, 201290.0, 1.5796, 200, 88228.0, 1.2482, 251, 58805.0, 1.8933, 316, 45507.0, 1.4742, 398, 43590.0, 1.5965, 501, 32343.0, 1.6862, 631, 27280.0, 1.5771, 794, 21915.0, 1.5811, 1000, 16628.0, 1.5833, 1259, 13473.0, 1.6077, 1587, 10860.0, 1.5543, 2000, 8253.0, 1.5837, 2512, 6753.5, 1.589, 3164, 5255.2, 1.5729, 4000, 4234.7, 1.5851, 5050, 3303.3, 1.5578, 6329, 2655.0, 1.5575, 8064, 2095.7, 1.5374, 10000, 1620.6, 1.5154, 12820, 1318.1, 1.5482, 16129, 1043.4, 1.5242, 20000, 954.93, 1.6984, 26315, 629.95, 1.5067, 33333, 376.08, 1.4983, 41666, 403.98, 1.422, 55555, 306.45, 1.4167, 71428, 232.99, 1.1215, 83333, 207.51, 1.4859]
    data10nFfull_calib_kelvin = [100, 108410.0, 3.2197, 126, 19777.0, -1.4326, 158, 66069.0, 1.85, 200, 114530.0, -4.3157, 251, 54210.0, -4.6172, 316, 26904.0, -4.4692, 398, 47435.0, -4.4398, 501, 31145.0, -4.6718, 631, 28871.0, -4.6088, 794, 18120.0, -4.7997, 1000, 16644.0, -4.6385, 1259, 12655.0, -4.6235, 1587, 10894.0, -4.6694, 2000, 8497.9, -4.6543, 2512, 6413.0, -4.7077, 3164, 5143.7, -4.6917, 4000, 4094.8, -4.7061, 5050, 3187.5, -4.7365, 6329, 2647.1, -4.707, 8064, 2149.9, -4.654, 10000, 1700.0, -4.6954, 12820, 1288.5, -4.6869, 16129, 1043.2, -4.7226, 20000, 818.23, -4.7975, 26315, 596.49, -4.7445, 33333, 500.17, 1.4598, 41666, 400.49, 1.3567, 55555, 280.65, -4.3433, 71428, 1052.1, 0.59461, 83333, 2948.3, 0.13815]
    # for i in range(len(data10nFfull_calib_kelvin)):
    #     if i % 3 == 2:
    #         data10nFfull_calib_kelvin[i] = -data10nFfull_calib_kelvin[i]
    data100Ohms_full_calib = [100, 24.771, 2.0709, 126, 183.12, -0.57313, 158, 82.134, -0.013541, 200, 97.633, 0.0070228, 251, 124.99, 0.19299, 316, 94.207, 0.027602, 398, 96.82, -0.042237, 501, 101.31, 0.20148, 631, 102.57, -0.0059444, 794, 102.06, 0.064538, 1000, 98.873, 0.0096382, 1259, 94.207, -0.01775, 1587, 99.394, 0.049771, 2000, 98.656, -0.0020811, 2512, 97.865, 0.0026471, 3164, 98.425, 0.010478, 4000, 98.908, 0.017963, 5050, 98.038, -0.016356, 6329, 97.478, -0.0059592, 8064, 99.634, -0.0072126, 10000, 98.016, -0.071563, 12820, 101.09, -0.0020346, 16129, 100.01, 0.0021159, 20000, 103.56, 0.33407, 26315, 99.186, -0.0091374, 33333, 77.686, -0.24319, 41666, 96.501, 0.0084937, 55555, 99.454, -0.014621, 71428, 132.98, 0.12362, 83333, 106.01, -0.33157]
    for i in range(len(data10nF)):
        if i % 3 == 2:
            data10nF[i] = - data10nF[i]

    # this one was done with a 10Hz starting point
    data100Ohms_full_calib_kelvin = [10, 116.82, 1.3012, 13, 125.4, -1.4474, 16, 127.03, -3.1368, 20, 139.9, -4.473, 25, 110.88, 3.041, 32, 112.47, 3.0925, 40, 142.95, -2.9774, 50, 85.208, 2.7298, 63, 167.33, 2.2214, 79, 122.39, -0.11696, 100, 18.721, -3.4266, 126, 422.62, 2.4082, 158, 112.74, -0.095284, 200, 100.1, -1.4718, 251, 153.8, -0.23304, 316, 91.6, 0.117, 398, 98.532, 0.042613, 501, 82.07, 0.15203, 631, 107.65, 0.035989, 794, 102.2, 0.02315, 1000, 114.25, -0.081407, 1259, 98.368, -0.0068832, 1587, 102.91, -0.015135, 2000, 100.59, 0.0087623, 2512, 93.935, 0.025244, 3164, 102.6, 0.06445, 4000, 98.691, 0.011291, 5050, 99.068, 0.0022699, 6329, 98.957, 0.0043945, 8064, 98.003, 0.0023239, 10000, 95.647, -0.072194, 12820, 99.263, -0.0055539, 16129, 97.223, -0.013207, 20000, 107.66, -0.1788, 26315, 98.47, -0.0078762, 33333, 126.16, 0.17279, 41666, 101.59, 0.029978, 55555, 99.396, 0.026728, 71428, 92.51, 0.11149, 83333, 97.144, 0.23794]
    data100Ohms_full_calib_kelvin2 = [100, 337.17, 0.0057128, 126, 180.45, -0.37359, 158, 103.1, -0.30037, 200, 31.345, -0.039895, 251, 105.96, -0.02882, 316, 129.18, 0.082421, 398, 130.91, 0.14883, 501, 103.87, -0.0075352, 631, 122.52, 0.019363, 794, 104.43, -0.29852, 1000, 104.38, -0.0015061, 1259, 99.493, -0.0014684, 1587, 102.27, 0.0064735, 2000, 103.97, 0.012588, 2512, 101.99, -0.014106, 3164, 102.17, 0.0074457, 4000, 103.5, -0.0036825, 5050, 105.59, -0.041673, 6329, 104.65, 0.0037417, 8064, 105.12, 0.05882, 10000, 106.11, 0.018506, 12820, 102.47, 0.020167, 16129, 104.11, -0.021621, 20000, 101.66, -0.071694, 26315, 98.94, -0.039762, 33333, 105.21, 6.1941, 41666, 112.8, 6.0441, 55555, 98.132, 0.34227, 71428, 535.87, 5.3355, 83333, 6518.1, 0.21146]
    data1kOhms_full_calib_kelvin = [10, 1147.9, -0.12665, 11, 1801.9, 6.1902, 12, 17707.0, -6.01, 13, 884.76, 2.917, 14, 760.49, 0.21207, 15, 6269.5, 7.0957, 16, 306.27, 2.9892, 17, 1898.0, 0.049572, 18, 1389.7, -3.2879, 20, 1074.9, 0.25928, 22, 1375.6, -3.9142, 23, 1429.4, -5.8866, 25, 577.76, 1.9751, 27, 1793.7, 1.3322, 29, 677.34, -5.6594, 32, 663.11, 1.2244, 34, 685.43, 1.5839, 37, 971.8, 1.0529, 40, 925.18, 1.1616, 43, 998.67, 0.94384, 46, 1028.1, 1.1072, 50, 1385.5, 1.2467, 54, 782.59, -4.8569, 58, 974.02, 0.69852, 63, 994.75, 0.95429, 68, 1254.9, 0.47204, 74, 1144.1, 0.15345, 79, 963.46, -0.084515, 86, 645.47, 0.25067, 93, 195.03, 0.93702, 100, 2620.6, 1.0113, 108, 1046.2, 0.91586, 117, 545.27, 0.94504, 126, 1669.1, 0.98767, 136, 852.82, -2.4392, 147, 488.3, -0.013531, 158, 915.69, -0.026358, 171, 994.34, 0.046044, 185, 1053.0, -0.045841, 200, 670.8, 0.1515, 215, 1257.0, -0.23934, 233, 994.8, -0.010724, 251, 1012.2, 0.021667, 271, 990.36, -0.44815, 293, 931.51, 0.00089058, 316, 1012.8, 0.042829, 341, 1083.8, -0.41346, 369, 769.99, 0.28216, 398, 962.92, 0.0042394, 430, 1283.1, -0.041581, 464, 897.18, 0.11196, 501, 871.28, -0.09284, 541, 958.36, 0.17914, 584, 1050.4, 0.1783, 631, 900.91, -0.07018, 681, 1054.0, -0.014155, 736, 988.8, 0.063975, 794, 991.32, -0.03914, 859, 1022.0, 0.012005, 927, 829.41, 0.02315, 1000, 893.27, -0.08968, 1082, 1031.6, -0.089391, 1168, 918.35, -0.055669, 1259, 904.46, 0.021577, 1362, 960.15, -0.021316, 1470, 977.53, 0.052847, 1587, 985.18, 0.010288, 1712, 915.31, -0.035563, 1851, 953.33, 0.021455, 2000, 974.84, -0.011318, 2155, 989.15, -0.0020087, 2336, 1002.9, -0.012454, 2512, 982.1, 0.032105, 2717, 967.76, 0.0063047, 2941, 1001.4, -0.022691, 3164, 955.7, 0.013065, 3424, 946.66, -0.0071223, 3703, 985.22, 0.030499, 4000, 965.84, 0.0089558, 4310, 973.08, 0.0021337, 4672, 950.83, -0.029256, 5050, 967.57, -0.0033269, 5434, 968.86, 0.017079, 5882, 975.02, -0.029605, 6329, 974.05, 0.0010114, 6849, 979.29, 5.6075e-05, 7462, 951.58, -0.013158, 8064, 951.15, 0.014849, 8620, 949.59, -0.011957, 9433, 972.83, 0.012715, 10000, 960.06, -0.023027, 10869, 957.38, 0.011501, 11904, 947.74, 0.0064741, 12820, 972.98, 0.01258, 13888, 969.35, 0.026263, 14705, 962.12, -0.0054972, 16129, 980.53, -0.0016293, 17241, 959.83, 0.010017, 18518, 933.34, 0.01429, 20000, 860.66, -0.012858, 21739, 961.81, -0.018962, 23809, 959.53, 0.0083921, 26315, 948.14, 0.0047983, 27777, 958.34, -0.0026509, 29411, 964.61, 0.001913, 33333, 870.25, 0.22371, 35714, 923.2, -0.00060498, 38461, 950.27, -0.0035662, 41666, 956.52, -0.015292, 45454, 957.43, 0.0031337, 50000, 1524.6, 0.53284, 55555, 984.46, 0.020829, 55555, 956.83, -0.065614, 62500, 960.11, -0.0058984, 71428, 1070.6, 6.2059, 71428, 982.17, 0.39146, 83333, 907.73, -0.021089, 83333, 1070.1, 0.25551, 100000, 11350.0, 1.657, 100000, 15207.0, 2.5387]

    dataOC = [100, 230170.0, 4.2067, 112, 100830.0, 4.8114, 126, 152990.0, 4.8784, 141, 99598.0, 5.3906, 158, 55178.0, 5.9649, 178, 133040.0, 4.915, 200, 72596.0, 5.3655, 224, 63965.0, 4.3501, 251, 46125.0, 4.8207, 282, 42033.0, 5.0181, 316, 25323.0, 4.8767, 355, 37063.0, 4.5037, 398, 35132.0, 4.8693, 447, 31146.0, 4.46, 501, 29216.0, 4.7915, 562, 22848.0, 4.7889, 631, 20672.0, 4.8091, 708, 18237.0, 4.6326, 794, 17065.0, 4.7549, 891, 13444.0, 4.7384, 1000, 12935.0, 4.7572, 1123, 10832.0, 4.7189, 1259, 9966.5, 4.7224, 1416, 9561.9, 4.7128, 1587, 8155.5, 4.6216, 1779, 7446.4, 4.7278, 2000, 6657.1, 4.6765, 2242, 6046.6, 4.6547, 2512, 5143.0, 4.7059, 2824, 4780.0, 4.7102, 3164, 4228.7, 4.7021, 3571, 3710.7, 4.7147, 4000, 3384.7, 4.7064, 4504, 3002.3, 4.6987, 5050, 2684.4, 4.6991, 5681, 2368.1, 4.6915, 6329, 2133.4, 4.6927, 7142, 1861.5, 4.688, 8064, 1659.9, 4.6754, 8928, 1484.2, 4.6832, 10000, 1336.1, 4.6761, 11363, 1167.7, 4.6699, 12820, 1046.0, 4.6604, 14285, 940.1, 4.6625, 16129, 830.77, 4.6524, 17857, 748.13, 4.6421, 20000, 666.77, 4.6393, 22727, 586.2, 4.6284, 26315, 506.88, 4.6154, 29411, 454.67, 4.6002, 33333, 400.83, 4.5887, 35714, 374.83, 4.5779, 41666, 321.4, 4.5562, 45454, 295.38, 4.5436, 55555, 241.89, 4.5047, 62500, 215.28, 4.4799, 71428, 186.88, 4.4443, 71428, 188.07, 4.4389, 83333, 162.08, 4.3994, 100000, 132.35, 4.3421]
    dataSC = [100, 131.86, 1.6564, 112, 34.486, 1.0101, 126, 56.265, 4.3218, 141, 117.03, 5.1129, 158, 127.7, 0.82989, 178, 111.24, 0.78032, 200, 66.364, 5.1114, 224, 17.663, 4.3924, 251, 27.552, 0.81195, 282, 46.006, 3.4039, 316, 31.822, 1.0373, 355, 4.3475, 4.8956, 398, 26.937, 6.261, 447, 12.428, 3.927, 501, 16.333, 5.9216, 562, 9.842, 1.8146, 631, 15.034, 5.7681, 708, 2.846, 2.1261, 794, 7.2099, 0.53531, 891, 9.6518, 1.4602, 1000, 9.0811, 4.865, 1123, 7.6712, 3.7647, 1259, 4.5494, 1.4101, 1416, 2.5591, 4.5052, 1587, 5.4824, 2.8167, 1779, 3.4197, 3.0001, 2000, 1.89, 0.37168, 2242, 2.4387, 5.0784, 2512, 3.6004, 1.4612, 2824, 1.8422, 5.709, 3164, 0.95602, 5.848, 3571, 2.4514, 0.62506, 4000, 1.6123, 3.0107, 4504, 1.8916, 2.3573, 5050, 1.2739, 2.1616, 5681, 1.4111, 0.32892, 6329, 0.86303, 2.4747, 7142, 0.71111, 0.80377, 8064, 0.24232, 2.9227, 8928, 0.95685, 3.1685, 10000, 0.78358, 4.7361, 11363, 0.69952, 1.5841, 12820, 0.38753, 5.156, 14285, 0.54875, 3.7731, 16129, 0.25506, 3.8445, 17857, 0.014297, 6.2393, 20000, 0.41035, 0.45997, 22727, 0.36397, 5.1334, 26315, 0.28681, 1.7488, 29411, 0.045076, 0.33785, 33333, 0.27456, 1.1151, 35714, 0.090757, 5.6374, 41666, 0.074768, 0.17119, 45454, 0.16134, 3.4105, 55555, 0.020648, 3.9522, 62500, 0.20822, 5.0936, 71428, 0.70612, 3.6996, 71428, 0.54396, 2.8353, 83333, 0.53574, 3.3734, 100000, 0.18507, 2.0764]
    # data = data1kOhms_full_calib_kelvin[3 * 45:-6]
    # data = data100Ohms_full_calib_kelvin2
    data = dataSC


    # freqs = []
    # mags = []
    # phases = []
    # for i in range(0, len(data)):
    #     if i % 3 == 0:
    #         freqs.append(data[i])
    #     elif i % 3 == 1:
    #         mags.append(data[i])
    #     elif i % 3 == 2:
    #         phases.append(data[i])

    main(data, freqlog=True)
