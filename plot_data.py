import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import scipy


parser = argparse.ArgumentParser(description='Plot sensor data')
parser.add_argument('--file', type=str, default='None', help='CSV file containing the sensor data')
args = parser.parse_args()

# Load the CSV file
file_name = args.file if args.file != 'None' else 'sensor_data_breath_stomach.csv'
df = pd.read_csv(file_name)

df.plot(subplots=True, figsize=(10, 8))
plt.show()
fs = 3573/30.0

# Frequency domain plot
fig = plt.figure(figsize=(10, 8))
plt.subplot(4, 1, 1)
plt.psd(df['Ax'], label='Ax', Fs=fs)
plt.ylabel("")
plt.legend()

plt.subplot(4, 1, 2)
plt.psd(df['Ay'], label='Ay', Fs=fs)
plt.ylabel("")
plt.legend()

plt.subplot(4, 1, 3)
plt.psd(df['Az'], label='Az', Fs=fs)
plt.ylabel("")
plt.legend()

plt.subplot(4, 1, 4)
plt.psd(df['Amag'], label='Amag', Fs=fs)
plt.ylabel("")
plt.legend()

fig.supylabel('Power Spectral Density (PSD) (dB/Hz)')
fig.supxlabel('Frequency (Hz)')
plt.show()

# b, a = scipy.signal.butter(4, (0.05, 0.78), fs=fs, btype='bandpass')

# filtered_ax = scipy.signal.filtfilt(b, a, df['Ax'])
# filtered_ay = scipy.signal.filtfilt(b, a, df['Ay'])
# filtered_az = scipy.signal.filtfilt(b, a, df['Az'])
# filtered_mag = scipy.signal.filtfilt(b, a, df['Amag'])

# plt.figure(figsize=(10, 8))
# plt.subplot(4, 1, 1)
# plt.psd(filtered_ax, label='Filtered Ax', Fs=fs)
# plt.ylabel("")
# plt.legend()

# plt.subplot(4, 1, 2)
# plt.psd(filtered_ay, label='Filtered Ay', Fs=fs)
# plt.ylabel("")
# plt.legend()

# plt.subplot(4, 1, 3)
# plt.psd(filtered_az, label='Filtered Az', Fs=fs)
# plt.ylabel("")
# plt.legend()

# plt.subplot(4, 1, 4)
# plt.psd(filtered_mag, label='Filtered Amag', Fs=fs)
# plt.ylabel("")
# plt.legend()

# fig.supylabel('Power Spectral Density (PSD) (dB/Hz)')
# fig.supxlabel('Frequency (Hz)')
# plt.show()


ax_down = scipy.signal.decimate(df['Ax'], 8)
ay_down = scipy.signal.decimate(df['Ay'], 8)
az_down = scipy.signal.decimate(df['Az'], 8)
mag_down = scipy.signal.decimate(df['Amag'], 8)

fig = plt.figure(figsize=(10, 8))
plt.subplot(4, 1, 1)
plt.psd(ax_down, label='Downsampled Ax', Fs=fs/8)
plt.ylabel("")
plt.legend()

plt.subplot(4, 1, 2)
plt.psd(ay_down, label='Downsampled Ay', Fs=fs/8)
plt.ylabel("")
plt.legend()

plt.subplot(4, 1, 3)
plt.psd(az_down, label='Downsampled Az', Fs=fs/8)
plt.ylabel("")
plt.legend()

plt.subplot(4, 1, 4)
plt.psd(mag_down, label='Downsampled Amag', Fs=fs/8)
plt.ylabel("")
plt.legend()

fig.supylabel('Power Spectral Density (PSD) (dB/Hz)')
fig.supxlabel('Frequency (Hz)')
plt.show()

# freqs = np.linspace(0, fs/8/2, len(az_down)//2 + 1)
# ind = np.argmax(az_down)
# print(np.linspace(0, fs/8/2, len(az_down)//2 + 1)[ind])

# FFT
spec = np.abs(np.fft.rfft(az_down)) ** 2

# frequency axis
freqs = np.fft.rfftfreq(len(az_down), d=1/(fs/8))   # fs/8 because you downsampled

# mask desired band
mask = (freqs >= 0.05) & (freqs <= 0.75)

# peak frequency in band
peak_freq = freqs[mask][np.argmax(spec[mask])]

print(f'Peak frequency: {peak_freq:.4f} Hz\nRespiratory Rate: {peak_freq*60:.4f} breaths/min')