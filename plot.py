import numpy as np
import plotly.graph_objects as go
import pandas as pd
import json
import os
import sys
import argparse
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
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if line:
                try:
                    # Skip incomplete lines (like just "}")
                    if line in ['{', '}', ',']:
                        print(f"Skipping incomplete line {line_num} in {filepath}: '{line}'")
                        continue
                    data.append(json.loads(line))
                except json.JSONDecodeError as e:
                    print(f"Skipping malformed JSON line {line_num} in {filepath}: {e}")
                    continue  # Skip malformed lines
    
    return pd.DataFrame(data)

def load_multiple_files(filepaths):
    """Load and combine multiple log files."""
    all_data = []
    
    for filepath in filepaths:
        print(f"Loading {filepath}...")
        try:
            df = load_log_file(filepath)
            if not df.empty:
                # Add source file column
                df['source_file'] = os.path.basename(filepath)
                all_data.append(df)
                print(f"  Loaded {len(df)} records")
            else:
                print(f"  No valid records found")
        except Exception as e:
            print(f"  Error loading {filepath}: {e}")
            continue
    
    if not all_data:
        raise ValueError("No valid data found in any files")
    
    # Combine all dataframes
    combined_df = pd.concat(all_data, ignore_index=True)
    
    # Sort by timestamp if available
    if 'gps_timestamp' in combined_df.columns:
        combined_df['timestamp'] = pd.to_datetime(combined_df['gps_timestamp'])
        combined_df = combined_df.sort_values('timestamp')
    
    print(f"Combined {len(combined_df)} total records from {len(all_data)} files")
    return combined_df

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
    
    # Handle different column names (old vs new format)
    if 'ppm_error' in df.columns:
        ppm_col = 'ppm_error'
    elif 'ppm_instantaneous' in df.columns:
        ppm_col = 'ppm_instantaneous'
    else:
        raise ValueError("No PPM error column found in data")
    
    # Convert PPM to PPB (1 ppm = 1000 ppb)
    ppb_error = df[ppm_col] * 1000
    
    # Create x-axis (use timestamp if available, otherwise sample number)
    if 'timestamp' in df.columns:
        x_data = df['timestamp']
        x_title = "Time"
    else:
        x_data = list(range(len(df)))
        x_title = "Sample Number"
    
    # Add original data
    fig.add_trace(go.Scatter(
        x=x_data,
        y=ppb_error, 
        name="PPB Error", 
        mode='markers',
        marker=dict(size=4, opacity=0.8),
        line=dict(color='blue')
    ))
    
    # Add filtered data if requested and we have enough data
    if show_filtered and len(ppb_error) > 10:
        ppb_error_filtered = apply_lowpass_filter(ppb_error, cutoff_freq=cutoff_freq)
        fig.add_trace(go.Scatter(
            x=x_data,
            y=ppb_error_filtered, 
            name="Filtered PPB Error", 
            mode='lines',
            line=dict(color='red', width=2)
        ))
        
        # Add normalized filtered data (centered around zero)
        ppb_mean = ppb_error.mean()
        ppb_error_normalized = ppb_error_filtered - ppb_mean
        fig.add_trace(go.Scatter(
            x=x_data,
            y=ppb_error_normalized, 
            name="Normalized PPB Error", 
            mode='lines',
            line=dict(color='green', width=2)
        ))
    
    # Calculate statistics
    ppb_mean = ppb_error.mean()
    ppb_std = ppb_error.std()
    
    # Update layout with cleaner titles
    fig.update_layout(
        title=f"Clock Source Stability Analysis (Mean: {ppb_mean:.2f} PPB, Std: {ppb_std:.2f} PPB)",
        xaxis_title=x_title,
        yaxis_title="Error (PPB)",
        showlegend=True,
        hovermode='x unified'
    )
    
    return fig

def main():
    parser = argparse.ArgumentParser(description='Plot frequency counter log files')
    parser.add_argument('files', nargs='+', help='JSONL log files to plot')
    parser.add_argument('--cutoff', type=float, default=0.001, help='Filter cutoff frequency (default: 0.001)')
    parser.add_argument('--no-filter', action='store_true', help='Disable filtering')
    parser.add_argument('--output', help='Save plot to HTML file instead of showing')
    
    args = parser.parse_args()
    
    # Load data from files
    if len(args.files) == 1:
        print(f"Loading single file: {args.files[0]}")
        df = load_log_file(args.files[0])
    else:
        print(f"Loading {len(args.files)} files...")
        df = load_multiple_files(args.files)
    
    if df.empty:
        print("No valid data found!")
        return
    
    print(f"Loaded {len(df)} total records")
    
    # Create plot
    fig = plot_ppm_analysis(df, show_filtered=not args.no_filter, cutoff_freq=args.cutoff)
    
    # Show or save plot
    if args.output:
        fig.write_html(args.output)
        print(f"Plot saved to {args.output}")
    else:
        fig.show()

if __name__ == "__main__":
    main()

