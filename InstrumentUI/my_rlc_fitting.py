import numpy as np
import matplotlib.pyplot as plt
import scipy
from scipy.optimize import curve_fit
import scipy.stats
import serial

# GLOBALS
# ser = serial.Serial('COM9', 115200)
# try:
#     ser.open()
# except serial.SerialException:
#     print('Port Cannot be Opened!')
#     while(1): pass


def series_model(freq, R, L, C):
    # if hasattr(freq, len):
    #     omega = 2*np.pi*np.array(freq)
    # else:
    #     omega = 2*np.pi*freq
    omega = 2*np.pi*freq
    return R + 1j * (omega * L - 1 / (omega * C))

def series_model_fit(freq, R, L, C):
    Z = series_model(freq, R, L, C)
    return np.concatenate([np.real(Z), np.imag(Z)])

def parallel_model(freq, R, L, C):
    # if hasattr(freq, len):
    #     omega = 2*np.pi*np.array(freq)
    # else:
    #     omega = 2*np.pi*freq
    omega = 2*np.pi*freq
    Y = 1/R - 1j*omega*C + 1 / (1j * omega * L)
    return 1/Y

def parallel_model_fit(freq, R, L, C):
    Z = parallel_model
    return np.concatenate([np.real(Z), np.imag(Z)])

def fit_model(func, freqs, measurements, guess):
    optimal_params, cov = curve_fit(func, freqs, measurements, guess)

    fit_error = measurements - func(*optimal_params)

    mse = np.mean(fit_error**2)

    std_err = np.sqrt(np.diag(cov))

    return optimal_params, mse, std_err


def fit_model_iterative(func, freqs, measurements, guess, Rtol=0.1, Ltol=1e-9, Ctol=1e-12,n_iter=20):
    current_guess = guess
    for _ in range(n_iter):
        opt_params, cov = curve_fit(func, freqs, measurements, current_guess)

        if( within_tolerance(*opt_params, *current_guess, Rtol, Ltol, Ctol) ):
            mse = np.mean( (measurements - func(freqs, *opt_params))**2 )
            std_err = np.sqrt(np.diag(cov))
            return opt_params, mse, std_err
        
        current_guess = opt_params

    mse = np.mean( (measurements - func(freqs, *opt_params))**2 )
    std_err = np.sqrt(np.diag(cov))

    return opt_params, mse, std_err

def within_tolerance(R1, L1, C1, R2, L2, C2, Rtol, Ltol, Ctol):
    if(np.abs(R1 - R2) < Rtol):
        return False
    if(np.abs(L1 - L2) < Ltol):
        return False
    if(np.abs(C1 - C2) < Ctol):
        return False
    return True

def main(measurements: dict[int, complex]) -> None:
    freqs = np.array(list(measurements.keys()))
    Z_measured = np.array(list(measurements.values()))
    # data passed in to the fit functions: 2xN dimension
    print(Z_measured)
    data = np.concatenate([np.real(Z_measured), np.imag(Z_measured)])
    print(f'data size: {data}')

    # compute initial guess
    R0 = np.mean(np.real(Z_measured))
    L0 = 1e-9
    C0 = 1e-9

    naive_guess = [R0, L0, C0]

    # iterative fit parameters
    # see about doing individual tolerances for RLC
    Rtol = 0.1
    Ltol = 1e-9
    Ctol = 1e-12
    n = 20

    # SERIES guesstimate
    optimal_params_series, mse_series, std_err_series = fit_model_iterative(
                                            series_model_fit, 
                                            freqs, 
                                            data, 
                                            naive_guess,
                                            Rtol=Rtol,
                                            Ltol=Ltol,
                                            Ctol=Ctol,
                                            n_iter=n
                                            )
    
    # PARALLEL guesstimate
    optimal_params_parallel, mse_parallel, std_err_parallel = fit_model_iterative(
                                            parallel_model_fit,
                                            freqs,
                                            data,
                                            naive_guess,
                                            Rtol=Rtol,
                                            Ltol=Ltol,
                                            Ctol=Ctol,
                                            n_iter=n
                                            )
    
    # compare
    if(mse_series < mse_parallel):
        best_model = 'series'
        best_params = optimal_params_series
        best_mse = mse_series
        best_std_err = std_err_series
        model_func = series_model
        model_fit_func = series_model_fit

    else:
        best_model = 'parallel'
        best_params = optimal_params_parallel
        best_mse = mse_parallel
        best_std_err = std_err_parallel
        model_func = parallel_model
        model_fit_func = parallel_model_fit

    # --- Display Results ---
    print("Best equivalent circuit model:", best_model)
    print("Fitted parameters (with 1-sigma error estimates):")
    print(f"R = {best_params[0]:.3e} ± {best_std_err[0]:.3e} Ohm")
    print(f"L = {best_params[1]:.3e} ± {best_std_err[1]:.3e} H")
    print(f"C = {best_params[2]:.3e} ± {best_std_err[2]:.3e} F")
    print(f"Sum of squared residuals: {best_mse:.3e}")
    
    # --- Plotting: Compare Measured Data with the Fitted Model ---
    freq_dense = np.linspace(min(freqs), max(freqs), 1000)
    Z_model_dense = model_func(freq_dense, *best_params)
    
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





if __name__ == '__main__':
    true_R = 100
    true_L = 1e-9
    true_C = 1e-12

    freqs = np.logspace(1, 6, 100).astype(int)

    measurements = {}

    for f in freqs:
        true_Z = series_model(f, true_R, true_L, true_C)
        complex_noise = (np.random.normal(loc=0, scale=0.05) * np.real(true_Z) + 
                    1j * np.random.normal(loc=0, scale=0.05) * np.imag(true_Z))
        
        measurements[f] = true_Z + complex_noise

    main(measurements)