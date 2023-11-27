 
import os
import numpy as np
from tqdm import tqdm
import matplotlib 
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator
from scipy import interpolate

params = {'xtick.labelsize': 24,
          'ytick.labelsize': 24,
          'font.size': 13,
          'figure.autolayout': True,  # similar to tight_layout
          'figure.figsize': [6.4, 4.8],  # ratio 1.6 is pleasant
          'axes.titlesize': 13,
          'axes.labelsize': 28,
          'lines.linewidth': 2,
          'lines.markersize': 4,
          'legend.fontsize': 16}

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)

coords = []

def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
    print (f'x = {ix}, y = {iy}')

    global coords
    coords.append((ix, iy))
    
    if len(coords) == 2:
        fig.canvas.mpl_disconnect(cid)

    return coords

files_list = []

tracker_event_path = "/usr/local/src/affine2dtracking/results/closed-loop-trans-x-added-latency"

for f in os.listdir(tracker_event_path):
    files_list.append(f)

sorted_files = sorted(files_list)

open("time_to_failure_tracker.txt", "w").close()

############################### event camera trans x (tracker side) ########################################################

mean_rate = []

for f in sorted_files:

    event_exp1_data = np.loadtxt(os.path.join(tracker_event_path, f), delimiter=" ", skiprows=3)[:,:11] 

    if "00ms" in f:

        time = event_exp1_data[:,0]
        n_events = event_exp1_data[:,2]

        rate = []
        for i in range(len(time)-1):
            if time[i+1]!=time[i]:
                rate.append(n_events[i]/(time[i+1]-time[i]))

        mean_rate.append(np.mean(rate))
        update_rate = event_exp1_data[:,8]
        eros_lat = event_exp1_data[:,9]

        freq = np.reciprocal(update_rate)

        # plt.figure()
        # plt.plot(time, update_rate)
        # plt.show()

        mean_update_rate = np.mean(update_rate)
        mean_latency = np.mean(update_rate+eros_lat)

        print(f + " "+str(1/mean_update_rate)+" Hz, "+str(mean_latency*1000)+" ms, EPS = "+str(np.mean(rate)))

speeds = ["V1","V2","V3","V4"]

eps = [mean_rate[0], mean_rate[1], mean_rate[2], mean_rate[3]]

fig, ax = plt.subplots(figsize=(15, 6))
plt.plot(speeds, eps)
ax.set_yscale('log')
plt.ylabel("Events per second")
plt.xlabel("Target speed [pxl/s]")
plt.show()