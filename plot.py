import numpy as np
import plotly.graph_objects as go
import pandas as pd
import json
import os
import sys
import argparse
from scipy.signal import butter, filtfilt, sosfiltfilt
import allantools


def plot_allan_deviation(df):

    ppm_col = 'ppm_instantaneous'
    ppb = np.array(df[ppm_col]) * 1000
    (tau, adev, adev_err, n) = allantools.oadev(ppb, rate=1.0, data_type="freq")
    fig = go.Figure()

    fig.add_trace(go.Scatter(
        x=tau,
        y=adev,
        mode="markers+lines",
        name="Allan deviation"
    ))

    fig.update_xaxes(title="Averaging Time τ (s)", type="log")
    fig.update_yaxes(title="Allan Deviation (ppb)", type="log")

    fig.update_layout(
        title="Allan Deviation",
        template="plotly_white"
    )
    return fig
import numpy as np
import plotly.graph_objects as go
import allantools

def plot_allan_deviation(df):
    ppm_col = 'ppm_instantaneous'
    ppb = np.array(df[ppm_col]) * 1000
    (tau, adev, adev_err, n) = allantools.oadev(ppb, rate=1.0, data_type="freq")

    fig = go.Figure()

    # main Allan deviation
    fig.add_trace(go.Scatter(
        x=tau,
        y=adev,
        mode="markers+lines",
        name="Allan deviation"
    ))

    # add reference slopes
    slopes = {
        "-1 (white phase noise)": -1,
        "-0.5 (flicker phase noise)": -0.5,
        "0 (white freq noise)": 0,
        "+0.5 (flicker freq noise)": 0.5,
        "+1 (random walk freq noise)": 1
    }

    # choose anchor near middle
    anchor_idx = len(tau) // 2
    anchor_tau = tau[anchor_idx]
    anchor_adev = adev[anchor_idx]

    tau_fit = np.logspace(np.log10(min(tau)), np.log10(max(tau)), 200)

    for label, slope in slopes.items():
        ref_line = anchor_adev * (tau_fit / anchor_tau) ** slope
        fig.add_trace(go.Scatter(
            x=tau_fit,
            y=ref_line,
            mode="lines",
            line=dict(dash="dot"),
            name=label
        ))

    fig.update_xaxes(title="Averaging Time τ (s)", type="log")
    fig.update_yaxes(title="Allan Deviation (ppb)", type="log")
    fig.update_layout(
        title="Allan Deviation with Reference Slopes",
        template="plotly_white"
    )
    return fig


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
    df = pd.DataFrame(data)
    df = df[(df['ticks'] >= 9_999_999) & (df['ticks'] <= 10_000_001)]
    return df

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

def plot_gps_map(df):
    """
    Create a map showing GPS coordinates and mean position.
    
    Args:
        df (pandas.DataFrame): DataFrame containing GPS data
    
    Returns:
        plotly.graph_objects.Figure: The created map figure
    """
    # Check if we have GPS data
    if 'gps_lat' not in df.columns or 'gps_lon' not in df.columns:
        return None
    
    # Filter out NaN values
    gps_data = df.dropna(subset=['gps_lat', 'gps_lon'])
    
    if len(gps_data) == 0:
        return None
    
    # Remove duplicate coordinates to reduce plot size
    gps_data_unique = gps_data.drop_duplicates(subset=['gps_lat', 'gps_lon'])
    
    # Calculate mean position from all data (not just unique points)
    mean_lat = gps_data['gps_lat'].mean()
    mean_lon = gps_data['gps_lon'].mean()
    
    print(f"GPS data points: {len(gps_data)} total, {len(gps_data_unique)} unique")
    print(f"Mean position: {mean_lat:.6f}, {mean_lon:.6f}")
    
    # Create map figure
    fig = go.Figure()
    
    # Add unique GPS points only
    fig.add_trace(go.Scattermapbox(
        lat=gps_data_unique['gps_lat'],
        lon=gps_data_unique['gps_lon'],
        mode='markers',
        marker=dict(
            size=8,
            color='blue',
            opacity=0.6
        ),
        name='GPS Points',
        text=[f"Point {i+1}" for i in range(len(gps_data_unique))],
        hovertemplate="<b>GPS Point %{text}</b><br>" +
                     "Lat: %{lat:.6f}<br>" +
                     "Lon: %{lon:.6f}<br>" +
                     "<extra></extra>"
    ))
    
    # Add mean position - try multiple approaches
    fig.add_trace(go.Scattermapbox(
        lat=[mean_lat],
        lon=[mean_lon],
        mode='markers',
        marker=dict(
            size=30,
            color='red',
            opacity=1.0
        ),
        name='Mean Position (RED CIRCLE)',
        text=['Mean'],
        hovertemplate="<b>Mean Position</b><br>" +
                     "Lat: %{lat:.6f}<br>" +
                     "Lon: %{lon:.6f}<br>" +
                     "<extra></extra>"
    ))
    
    # Add another marker with different symbol to see what works
    fig.add_trace(go.Scattermapbox(
        lat=[mean_lat],
        lon=[mean_lon],
        mode='markers',
        marker=dict(
            size=25,
            color='yellow',
            symbol='circle',
            opacity=1.0
        ),
        name='Mean Position (YELLOW)',
        text=['Mean2'],
        hovertemplate="<b>Mean Position (Yellow)</b><br>" +
                     "Lat: %{lat:.6f}<br>" +
                     "Lon: %{lon:.6f}<br>" +
                     "<extra></extra>"
    ))
    
    # Update layout for mapbox
    fig.update_layout(
        mapbox=dict(
            style="open-street-map",
            center=dict(lat=mean_lat, lon=mean_lon),
            zoom=15
        ),
        title=f"GPS Track (Mean: {mean_lat:.6f}, {mean_lon:.6f})",
        showlegend=True,
        height=600
    )
    
    return fig

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
    
    # Create x-axis using GPS timestamp if available
    if 'gps_timestamp' in df.columns:
        # Convert GPS timestamp to datetime
        x_data = pd.to_datetime(df['gps_timestamp'])
        x_title = "GPS Time (UTC)"
    elif 'timestamp' in df.columns:
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
#        fig.add_trace(go.Scatter(
#            x=x_data,
#            y=ppb_error_normalized, 
#            name="Normalized PPB Error", 
#            mode='lines',
#            line=dict(color='green', width=2)
#        ))
    
    # Calculate statistics
    ppb_mean = ppb_error.mean()
    ppb_std = ppb_error.std()
    ppm_mean = ppb_mean / 1000
    
    # Calculate updated offset if oscillator_offset_ppm exists
    offset_info = ""
    if 'oscillator_offset_ppm' in df.columns:
        current_offset = df['oscillator_offset_ppm'].iloc[0]  # Should be constant
        updated_offset = current_offset + ppm_mean
        offset_info = f" | Current Offset: {current_offset:.6f} ppm | Updated Offset: {updated_offset:.6f} ppm"
    
    # Update layout with cleaner titles
    fig.update_layout(
        title=f"Clock Source Stability Analysis (Mean: {ppb_mean:.2f} PPB, Std: {ppb_std:.2f} PPB{offset_info})",
        xaxis_title=x_title,
        yaxis_title="Error (PPB)",
        showlegend=True,
        hovermode='x unified',
        yaxis_range=[-11, 11],
    )
    
    return fig

def create_combined_html(freq_fig, allan_fig, map_fig=None, output_file='frequency_analysis.html'):
    """
    Create a single HTML file with all plots embedded.
    
    Args:
        freq_fig: Frequency analysis plot
        allan_fig: Allan deviation plot
        map_fig: GPS map plot (optional)
        output_file: Output filename
    """
    from plotly.offline import plot
    import plotly.io as pio
    
    # Convert plots to HTML divs
    freq_html = plot(freq_fig, output_type='div', include_plotlyjs=False)
    allan_html = plot(allan_fig, output_type='div', include_plotlyjs=False)
    
    map_section = ""
    if map_fig:
        map_html = plot(map_fig, output_type='div', include_plotlyjs=False)
        map_section = f"""
    <div class="plot-section">
        <h2>GPS Track</h2>
        {map_html}
    </div>"""
    
    # Create combined HTML
    combined_html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Frequency Counter Analysis</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body {{ 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background-color: #f8f9fa;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        .plot-section {{ 
            margin: 30px 0; 
            padding: 20px;
            border: 1px solid #e0e0e0;
            border-radius: 6px;
            background-color: #fafafa;
        }}
        h1 {{ 
            color: #2c3e50; 
            text-align: center;
            border-bottom: 3px solid #3498db; 
            padding-bottom: 15px;
            margin-bottom: 30px;
        }}
        h2 {{ 
            color: #34495e; 
            border-bottom: 2px solid #bdc3c7; 
            padding-bottom: 10px;
            margin-top: 0;
        }}
        .plotly-graph-div {{
            border-radius: 4px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>Frequency Counter Analysis</h1>
        
        <div class="plot-section">
            <h2>Frequency Stability Analysis</h2>
            {freq_html}
        </div>
        
        <div class="plot-section">
            <h2>Allan Deviation Analysis</h2>
            {allan_html}
        </div>
        {map_section}
    </div>
</body>
</html>
"""
    
    with open(output_file, 'w') as f:
        f.write(combined_html)
    
    print(f"Combined analysis saved to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Plot frequency counter log files')
    parser.add_argument('files', nargs='+', help='JSONL log files to plot')
    parser.add_argument('--cutoff', type=float, default=0.001, help='Filter cutoff frequency (default: 0.001)')
    parser.add_argument('--no-filter', action='store_true', help='Disable filtering')
    parser.add_argument('--output', default='frequency_analysis.html', help='Output HTML file (default: frequency_analysis.html)')
    parser.add_argument('--no-map', action='store_true', help='Disable GPS map')
    
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
    
    # Create all plots
    freq_fig = plot_ppm_analysis(df, show_filtered=not args.no_filter, cutoff_freq=args.cutoff)
    allan_fig = plot_allan_deviation(df)
    
    # Create GPS map if requested and available
    map_fig = None
    if not args.no_map:
        map_fig = plot_gps_map(df)
        if not map_fig:
            print("No GPS data found for mapping")
    
    # Create combined HTML file
    create_combined_html(freq_fig, allan_fig, map_fig, args.output)

if __name__ == "__main__":
    main()

