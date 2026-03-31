#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def analyze_frequency(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: The file {csv_path} does not exist.")
        return

    # Load data
    df = pd.read_csv(csv_path)
    
    # Assume the time column is named 'time' or fallback to the first column
    time_col = 'time' if 'time' in df.columns else df.columns[0]
    
    # Calculate average time step (dt) and sampling frequency (fs)
    dt = df[time_col].diff().mean()
    fs = 1.0 / dt
    print(f"Analysis started. Detected sampling frequency: {fs:.2f} Hz")

    # Select columns to analyze (e.g., accelerations and angular velocities)
    cols_to_analyze = [c for c in df.columns if 'acc' in c.lower() or 'omega' in c.lower()]
    
    if not cols_to_analyze:
        print("No acceleration or angular velocity columns found.")
        return

    # Create a plot with a specific size
    plt.figure(figsize=(12, 8))

    # Limit to the first 4 columns to avoid overcrowding the plot
    for i, col in enumerate(cols_to_analyze[:4]): 
        signal = df[col].values
        n = len(signal)
        
        # Detrend the signal (remove DC offset/mean) to highlight actual oscillations
        signal_detrended = signal - np.mean(signal)

        # Compute the Fast Fourier Transform (FFT)
        # rfft is optimized for real-valued signals, returning only positive frequencies
        frequencies = np.fft.rfftfreq(n, d=dt)
        fft_values = np.abs(np.fft.rfft(signal_detrended))

        # 1. Plot Time Domain (Raw Signal)
        plt.subplot(len(cols_to_analyze[:4]), 2, 2*i + 1)
        plt.plot(df[time_col], signal)
        plt.title(f"Time Domain: {col}")
        plt.grid(True)
        plt.ylabel("Amplitude")
        if i == len(cols_to_analyze[:4]) - 1:
            plt.xlabel("Time [s]")

        # 2. Plot Frequency Domain (FFT Spectrum)
        plt.subplot(len(cols_to_analyze[:4]), 2, 2*i + 2)
        plt.plot(frequencies, fft_values)
        plt.title(f"Frequency Domain (FFT): {col}")
        plt.grid(True)
        # Display frequencies up to the Nyquist limit (fs/2)
        plt.xlim(0, fs / 2.0) 
        # Use logarithmic scale to make high-frequency noise visible
        plt.yscale('log') 
        if i == len(cols_to_analyze[:4]) - 1:
            plt.xlabel("Frequency [Hz]")

    # Adjust layout to prevent overlap and show the plot
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Allow passing the CSV path as a command line argument or via input prompt
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = input("Enter the path to the CSV file: ")
    
    analyze_frequency(path)