import numpy as np
import scipy
import pandas as pd
import matplotlib.pyplot as plt
import argparse


parser = argparse.ArgumentParser(description='Load sensor data')
parser.add_argument('--file', type=str, default='None', help='CSV file containing the sensor data')
args = parser.parse_args()

# Load the CSV file
# file_name = args.file if args.file != 'None' else 'dual_sensor_data.csv'
# df = pd.read_csv(file_name)
FS = 2684/30.0 # Effective sampling frequency (total samples / duration in seconds)

b, a = scipy.signal.butter(2, [10, 40], btype='bandpass', fs=FS)

def calc_RR(df): 
    # calculate respiratory rate given accelerometer data from both sensors (xiphoid and suprasternal)
    # down sample by factor of 6 to get ~15 Hz sampling rate
    az_down_xiphoid = scipy.signal.decimate(df['Az'], 6)
    az_down_supra = scipy.signal.decimate(df['Az2'], 6)

    # get power spectrum
    spec_xiphoid = np.abs(np.fft.rfft(az_down_xiphoid)) ** 2
    spec_supra = np.abs(np.fft.rfft(az_down_supra)) ** 2

    # frequency axis
    freqs_xiphoid = np.fft.rfftfreq(len(az_down_xiphoid), d=1/(FS/6))   # fs/6 because you downsampled
    freqs_supra = np.fft.rfftfreq(len(az_down_supra), d=1/(FS/6))   # fs/6 because you downsampled

    # mask desired band for RR
    mask_xiphoid = (freqs_xiphoid >= 0.05) & (freqs_xiphoid <= 0.78)
    mask_supra = (freqs_supra >= 0.05) & (freqs_supra <= 0.78)

    # peak frequency in band
    peak_freq_xiphoid = freqs_xiphoid[mask_xiphoid][np.argmax(spec_xiphoid[mask_xiphoid])]
    peak_freq_supra = freqs_supra[mask_supra][np.argmax(spec_supra[mask_supra])]

    # convert to breaths per minute
    RR_xiphoid = peak_freq_xiphoid * 60
    RR_supra = peak_freq_supra * 60

    return (RR_xiphoid + RR_supra) / 2  # Average of the two sensors


def bandpower(df):
    az_down_xiphoid = scipy.signal.decimate(df['Az'], 6)
    az_down_supra = scipy.signal.decimate(df['Az2'], 6)

    # get power spectrum
    spec_xiphoid = np.abs(np.fft.rfft(az_down_xiphoid)) ** 2
    spec_supra = np.abs(np.fft.rfft(az_down_supra)) ** 2

    # frequency axis
    freqs_xiphoid = np.fft.rfftfreq(len(az_down_xiphoid), d=1/(FS/6))   # fs/6 because you downsampled
    freqs_supra = np.fft.rfftfreq(len(az_down_supra), d=1/(FS/6))   # fs/6 because you downsampled

    # mask desired band for RR
    mask_xiphoid = (freqs_xiphoid >= 0.05) & (freqs_xiphoid <= 0.78)
    mask_supra = (freqs_supra >= 0.05) & (freqs_supra <= 0.78)


    return np.trapz(spec_xiphoid[mask_xiphoid], freqs_xiphoid[mask_xiphoid]), np.trapz(spec_supra[mask_supra], freqs_supra[mask_supra])

def not_worn(df):
    bandpower_xiphoid, bandpower_supra = bandpower(df)
    if min(bandpower_xiphoid, bandpower_supra) < 1:
        return True
    return False


def calc_HR(df, fs, cols=['Ax', 'Ay']):
    eps = 1e-10

    # 1. Euclidean norm
    x = np.sqrt((df[cols]**2).sum(axis=1))

    # 2. Bandpass
    x = scipy.signal.filtfilt(b, a, x)

    # 3. Shannon energy
    x = np.abs(x) + eps
    x /= np.max(np.abs(x))
    x = -x * np.log(x)

    # 4. Smooth with moving average
    win = int(0.2 * fs)
    x = np.convolve(x, np.ones(win)/win, mode='same')

    # 5. Hilbert transform (simple version)
    h = np.imag(scipy.signal.hilbert(x))

    # 6. Positive zero crossings = peaks
    peaks = np.where((h[:-1] < 0) & (h[1:] >= 0))[0]

    # 7. HR in 2s windows
    window = int(2 * fs)
    hr = []

    for i in range(0, len(x) - window, window):
        count = np.sum((peaks >= i) & (peaks < i + window))
        hr.append((count / 2) * 60)

    return sum(hr) / len(hr)




    # def enforce_refractory(peaks, fs, min_interval=0.3):
    #     min_samples = int(min_interval * fs)
    #     filtered = []

    #     last = -np.inf
    #     for p in peaks:
    #         if p - last > min_samples:
    #             filtered.append(p)
    #             last = p

    #     return np.array(filtered)
    
    # peaks = enforce_refractory(peaks, fs)


def find_peaks(df, fs, cols=['Ax', 'Ay']):
    eps = 1e-10

    # 1. Euclidean norm
    x = np.sqrt((df[cols]**2).sum(axis=1))

    # 2. Bandpass
    x = scipy.signal.filtfilt(b, a, x)

    # 3. Shannon energy
    x = np.abs(x) + eps
    x /= np.max(np.abs(x))
    x = -x * np.log(x)

    # 4. Smooth
    win = int(0.2 * fs)
    x = np.convolve(x, np.ones(win)/win, mode='same')

    # 5. Hilbert transform (simple version)
    h = np.imag(scipy.signal.hilbert(x))

    # 6. Positive zero crossings = peaks
    peaks = np.where((h[:-1] < 0) & (h[1:] >= 0))[0]

    return peaks

def calc_PTT(df, fs):
    xiph_peaks = find_peaks(df, fs)
    supra_peaks = find_peaks(df, fs, cols = ['Ax2', 'Ay2'])  
    window = int(2 * fs)
    ptt = []

    for i in range(0, len(df) - window, window):
        window_xiph = xiph_peaks[(xiph_peaks >= i) & (xiph_peaks < i + window)]
        window_supra = supra_peaks[(supra_peaks >= i) & (supra_peaks < i + window)]
        l = min(len(window_xiph), len(window_supra))
        ptt.append(np.mean((window_supra[:l] - window_xiph[:l]) * 1 / fs))

    avg_ptt = sum(ptt) / len(ptt)
    return avg_ptt


 # def enforce_refractory(peaks, fs, min_interval=0.3):
    #     min_samples = int(min_interval * fs)
    #     filtered = []

    #     last = -np.inf
    #     for p in peaks:
    #         if p - last > min_samples:
    #             filtered.append(p)
    #             last = p

    #     return np.array(filtered)
    
    # peaks = enforce_refractory(peaks, fs)  


if __name__ == "__main__":
    #Load the CSV file
    for i in range(1, 11):
        file_name = f'dual_sensor_data{i}.csv'
        df = pd.read_csv(file_name)
        print(f"Estimated Respiratory Rate from {file_name}: {calc_RR(df):.2f} breaths per minute")

    for i in range(1, 11):
        file_name = f'dual_sensor_data{i}.csv'
        df = pd.read_csv(file_name)
        print(f"Estimated Heart Rate from {file_name}: {calc_HR(df, fs=len(df)/30.0).mean():.2f} beats per minute")    

    BPs = [118, 111, 109, 114, 107, 118, 110, 115, 115, 114]
    ptts = []

    for i in range(1, 11):
        file_name = f'dual_sensor_data{i}.csv'
        df = pd.read_csv(file_name)
        FS = len(df) / 30.0 # Effective sampling frequency (total samples / duration in seconds)
        b, a = scipy.signal.butter(4, [10, 40], btype='bandpass', fs=FS)
        ptt = calc_PTT(df, fs=FS)
        ptts.append(ptt)

    plt.scatter(ptts, BPs)
    plt.xlabel('Pulse Transit Time (s)')
    plt.ylabel('Blood Pressure (mmHg)')
    m, b = np.polyfit(ptts, BPs, 1)
    plt.plot(ptts, m * np.array(ptts) + b, color='black')
    plt.show()

    # file_names = ['sensor_data_idle4.csv', 'sensor_data_idle5.csv']
    # for file_name in file_names:
    #     df = pd.read_csv(file_name)
    #     print(f"Estimated Respiratory Rate from {file_name}: {calc_RR(df):.2f} breaths per minute")

    # for i in range(1, 11):
    #     file_name = f'dual_sensor_data{i}.csv'
    #     print(file_name, not_worn(pd.read_csv(file_name)))

    # for i in range(4, 10):
    #     file_name = f'sensor_data_idle{i}.csv'
    #     print(file_name, not_worn(pd.read_csv(file_name)))

    # file_name = f'sensor_data_idle8.csv'
    # df = pd.read_csv(file_name)
    # print(file_name, bandpower(df))

    # file_name = f'dual_sensor_data1.csv'
    # df = pd.read_csv(file_name)
    # print(file_name, calc_HR(df, fs=len(df)/30.0))