"""
Butterworth Low-pass Filter
===========================
"""

# import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, optimize

from scipy.optimize import curve_fit, least_squares
from scipy.signal import medfilt
# from scipy import signal, optimize
from scipy.fftpack import rfft, irfft, rfftfreq
from sklearn.linear_model import LinearRegression


# Butterworth filter
def data_filter(data_measured, order, cutoff):
    """
    creates a 3. order Butterworth lowpass filter with a cutoff of 0.2 times
    the Nyquist frequency or 200 Hz, returning enumerator (b) and
    denominator (a) polynomials for a Infinite Impulse Response (IIR) filter
    """
    b, a = signal.butter(order, cutoff)

    # applies a linear digital filter twice, once forward and once backwards.
    # The combined filter has zero phase and a filter order twice that of the original.
    data_filtered = signal.filtfilt(b, a, data_measured)
    return data_filtered




""""
The result should be approximately xlow, with no phase shift.

>>> b, a = signal.butter(8, 0.125)
>>> y = signal.filtfilt(b, a, x, padlen=150)
>>> np.abs(y - xlow).max()
9.1086182074789912e-06



    data_filtered = np.zeros(n)
    i = 0

    # choose an alpha value between 0 and 1, where 1 is equivalent to
    # unfiltered data
    while i < n:
        data_filtered[i] = data_measured[i] * alpha + (data_filtered * (1.0 - alpha))
        i += 1
    return data_filtered
"""
