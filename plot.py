import numpy as np
import plotly.graph_objects as go
import pandas as pd
import json
import os
from scipy.signal import butter, filtfilt, sosfiltfilt

def load_log_file(filepath):
    """
    Load a frequency counter log file as a pandas DataFrame.
    
    Supports both JSON format (old format) and CSV format (new format from Arduino).
    
    Args:
        filepath (str): Path to the log file
        
    Returns:
        pandas.DataFrame: DataFrame with columns:
            - For JSON format: timestamp_ms, ticks, freq_hz, freq_mhz, ppm_error, ppm_avg
            - For CSV format: elapsed_sec, latest_mhz, avg_mhz, ppm_instantaneous, ppm_average, total_ticks, ticks, sample_count
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Log file not found: {filepath}")
    
    # Read first few lines to determine format
    with open(filepath, 'r') as f:
        first_line = f.readline().strip()
    
    # Check if it's JSON format (starts with {)
    if first_line.startswith('{'):
        return load_json_log(filepath)
    # Check if it's CSV format (starts with # or contains commas)
    elif first_line.startswith('#') or ',' in first_line:
        return load_csv_log(filepath)
    else:
        raise ValueError(f"Unknown log file format: {filepath}")

def load_json_log(filepath):
    """Load JSON format log file."""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    data.append(json.loads(line))
                except json.JSONDecodeError:
                    continue  # Skip malformed lines
    
    return pd.DataFrame(data)

def apply_lowpass_filter(data, sampling_rate=1.0, cutoff_freq=0.01, filter_order=8, pad_length=2000):
    """
    Apply a Butterworth low-pass filter to the input data using SOS format.
    
    Args:
        data (array-like): Input data to filter
        sampling_rate (float): Sampling rate in Hz (default: 1.0 for 1 PPS)
        cutoff_freq (float): Cutoff frequency in Hz (default: 0.01)
        filter_order (int): Filter order (default: 8)
        pad_length (int): Number of samples to pad on each side (default: 2000)
    
    Returns:
        numpy.ndarray: Filtered data using zero-phase filtering
    """
    # Calculate normalized cutoff frequency
    nyquist = sampling_rate / 2
    normal_cutoff = cutoff_freq / nyquist
    
    # Design Butterworth filter in SOS format
    sos = butter(filter_order, normal_cutoff, btype='low', analog=False, output='sos')
    
    # Pad data with mean value on both sides to reduce edge effects
    data_mean = np.mean(data)
    padded_data = np.pad(data, pad_length, mode='constant', constant_values=data_mean)
    
    # Apply filter using sosfiltfilt (zero-phase filtering with SOS)
    filtered_padded = sosfiltfilt(sos, padded_data)
    
    # Remove padding to return original length
    filtered_data = filtered_padded[pad_length:-pad_length]
    
    return filtered_data

def plot_ppm_analysis(df, show_filtered=True, cutoff_freq=0.1):
    """
    Create a plot showing frequency error analysis with optional filtering in PPB.
    
    Args:
        df (pandas.DataFrame): DataFrame containing frequency data
        show_filtered (bool): Whether to show filtered data (default: True)
        cutoff_freq (float): Cutoff frequency for filtering (default: 0.1 Hz)
    
    Returns:
        plotly.graph_objects.Figure: The created plot figure
    """
    fig = go.Figure()
    
    # Convert PPM to PPB (1 ppm = 1000 ppb)
    ppb_error = df['ppm_error'] * 1000
    
    # Add original data
    fig.add_trace(go.Scatter(
        y=ppb_error, 
        name="Original PPB Error", 
        mode='markers',
        marker=dict(size=4, opacity=0.6),
        line=dict(color='lightblue')
    ))
    
    # Add filtered data if requested
    if show_filtered:
        ppb_error_filtered = apply_lowpass_filter(ppb_error, cutoff_freq=cutoff_freq)
        fig.add_trace(go.Scatter(
            y=ppb_error_filtered, 
            name="Filtered PPB Error", 
            mode='lines',
            line=dict(color='red', width=2)
        ))
        
        # Add normalized filtered data (centered around zero)
        ppb_mean = ppb_error.mean()
        ppb_error_normalized = ppb_error_filtered - ppb_mean
        fig.add_trace(go.Scatter(
            y=ppb_error_normalized, 
            name="Normalized PPB Error", 
            mode='lines',
            line=dict(color='green', width=2)
        ))
    
    # Calculate mean for title
    ppb_mean = ppb_error.mean()
    
    # Update layout with cleaner titles
    fig.update_layout(
        title=f"Clock Source Stability Analysis (Mean: {ppb_mean:.2f} PPB)",
        xaxis_title="Sample Number",
        yaxis_title="Error (PPB)",
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01
        ),
        hovermode='x unified'
    )
    
    return fig

df = load_json_log("log.txt")
fig = plot_ppm_analysis(df, cutoff_freq=0.001)
fig.show()

