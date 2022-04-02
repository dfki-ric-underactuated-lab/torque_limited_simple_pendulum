"""
Fast Fourier Transform
======================
"""


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import rfft, irfft, rfftfreq


# Fast Fourier Transform (FFT)
def fast_fourier_transform(data_measured, data_desired, n, t):
    """
    Fast Fourier transform method
    """
    # Define function on which the FFT shall be executed                         
    dm_pos = data_measured["pos"]                         
    dm_vel = data_measured["vel"]
    dm_tau = data_measured["torque"]
    dml = [dm_pos, dm_vel, dm_tau]                                          # dml is of type list and will later be reconverted to type dataframe
                          
    dd_pos = data_desired["pos"]                         
    dd_vel = data_desired["vel"]
    dd_tau = data_desired["torque"]
    ddl = [dd_pos, dd_vel, dd_tau]                                          # ddl is of type list and will later be reconverted to type dataframe

    # Compute the FFT
    dm_hat = np.fft.fft(dml, n)

    # Compute the power spectrum density for pos, vel, tau
    psd_measured = dm_hat * np.conj(dm_hat) / n                             # power spectrum density

    # Use the PSD to filter out noise 
    bounds = [30, 100, 300]                                                 # frequencies with lower PSD than bounds get cut off
    indices = np.empty_like(psd_measured)
    for i in range(len(indices)):
        indices[i] = psd_measured[i] > bounds[i]                            # find all freqs with large power
    psd_filtered = psd_measured * indices                                   # zero out all others
    dm_hat = indices * dm_hat                                               # zero out small Fourier coeffs. in Y
    f = np.fft.ifft(dm_hat)                                                 # inverse FFT for filtered time signal

    # Convert lists back to dataframes
    dm_t = pd.DataFrame(dml)                                                # convert lists dml, ddl and f back to type dataframe
    dd_t = pd.DataFrame(ddl)
    df_t = pd.DataFrame(f)
    dm = dm_t.T                                                             # transpose dataframes
    dd = dd_t.T
    df = df_t.T
    dm.insert(0, "time", t, True)                                           # insert new time column into dataframes
    dd.insert(0, "time", t, True)
    df.insert(0, "time", t, True)
    df.columns = ['time', 'pos', 'vel', 'torque']                           # rename columns of df

    return dm, dd, df, psd_measured, psd_filtered


# FFT scipy alternative
def scipy_fft(data_measured, smooth_freq=100):
    """
    SciPy alternative
    """
    w = rfft(data_measured)                    # FFT where data_measured is an array
    spectrum = w**2                            # squared amplitudes w of the FFT
    # cuts off frequencies that are much lower (/smooth_freq) then the main frequency at spectrum.max.()
    cutoff_idx = spectrum < (spectrum.max() / smooth_freq)
    w2 = w.copy()
    w2[cutoff_idx] = 0
    # inverses the filtered FFT values back to the original data type
    data_filtered = irfft(w2)

    return data_filtered

