import numpy as np
import plotly.graph_objects as go

ppms = []
with open ("screenlog.0") as f:
    for line in f.readlines():
        if "ppm(avg)" not in line:
            continue
        line = line.strip()
        line = line.split(" ")
        
        lat_avg = float(line[-6].split("=")[-1]), float(line[-4].split("=")[-1])
        lat_avg = float(line[-2].split("=")[-1]), float(line[-1].split("=")[-1])
        ppms.append(lat_avg)
ppms = np.array(ppms)

fig = go.Figure()
fig.add_trace(go.Scatter(y=ppms[:,0], name="latest", mode='lines+markers'))
fig.add_trace(go.Scatter(y=ppms[:,1], name="average"))
fig.update_layout(
    title="clock source stability",
    xaxis_title="Averaging Time (s)",
    yaxis_title="Error (ppm)",
)
fig.show()
