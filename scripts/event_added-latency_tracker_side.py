 
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

for f in sorted_files:

    event_exp1_data = np.loadtxt(os.path.join(tracker_event_path, f), delimiter=" ", skiprows=3)[:,:11] 
    delta_u = event_exp1_data[:,3]-320

    fig = plt.figure()
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    plt.plot(event_exp1_data[:,0], delta_u, label='u')
    plt.xlabel('Time [s]')
    plt.ylabel('State')
    plt.title('Target speed ='+f)
    plt.legend()
    plt.show() 

    with open('time_to_failure_tracker_side.txt', 'a') as file:
        if coords[0][0] > 10:
            mean_error = np.mean(abs(delta_u)) 
            std_error = np.std(abs(delta_u))
            file.write(f + " " + "NO FAILURE, mean="+ str(mean_error) + ", std="+ str(std_error)+ "\n")   
        else:
            mean_error = np.mean(abs(delta_u)) 
            file.write(f + " FAILED AT " + str(coords[0][0])+ " "+ str(mean_error)+"\n")

    coords = []