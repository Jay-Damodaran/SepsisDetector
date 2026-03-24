import csv
import matplotlib.pyplot as plt
import pandas as pd
import serial
import time
import os

# csv file saved next to script
script_dir = os.path.dirname(os.path.abspath(__file__))
file_name = os.path.join(script_dir, "sensor_data_breath_stomach4.csv")

# Change 'COM3' to your Arduino's port
ser = serial.Serial('COM6', 115200)
time.sleep(2) # Wait for the serial connection to initialize

print("Logging started... Press Ctrl+C to stop.")
started = False
with open(file_name, "a", newline="") as f:
    writer = csv.writer(f)
    t0 = time.perf_counter() # Start timer
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line == 'start' and not started:
                t0 = time.perf_counter()
                print("Logging started... Press Ctrl+C to stop.")
                started = True
            elif not started:
                continue
            elif line:
                data = line.split(",")
                print(f"Saving: {data}")
                writer.writerow(data)
                f.flush() # Forces data into the file immediately
            if (time.perf_counter() - t0) >= 30: # Stop after 30 seconds
                print("Logging stopped after 30 seconds.")
                break
        except KeyboardInterrupt:
            print("Logging stopped.")
            break

ser.close()


df = pd.read_csv(file_name)
print(df.head())

