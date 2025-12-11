import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# Create a noisy signal
t = np.linspace(0, 1, 1000, False)  # 1 second
sig = np.sin(2*np.pi*10*t) + np.sin(2*np.pi*20*t) + 0.5*np.random.randn(len(t))
sig_f = 0  # initial filtered signal value

f_c = 10
f_s = 100
alpha = 1 - np.exp(-2 * np.pi * f_c / f_s) # 越小越平滑但反應越慢
sig_f = alpha * sig + (1 - alpha) * sig_f  # simulate some noise

# Design the Butterworth filter
order = 2  # Filter order
cutoff = 15  # Cutoff frequency in Hz
order_1 = 3
cutoff_1 = 15
b, a = signal.butter(order, cutoff, 'low', fs=100)
b1, a1 = signal.butter(order_1, cutoff_1, 'low', fs=100)

# Apply the filter
filtered_sig = signal.filtfilt(b, a, sig)
filtered_sig_1 = signal.filtfilt(b1, a1, sig)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(t, sig, 'b-', label='Noisy signal')
plt.plot(t, sig_f, 'k-', label='Single-pole IIR filtered signal')
plt.plot(t, filtered_sig, 'r-', label='Filtered signal (order 2, cutoff 5 Hz)')
plt.plot(t, filtered_sig_1, 'g-', label='Filtered signal (order 3, cutoff 5 Hz)')
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.title('Butterworth Low-Pass Filter')
plt.grid()
plt.show()