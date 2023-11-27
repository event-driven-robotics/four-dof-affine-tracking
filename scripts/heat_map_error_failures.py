 
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

errors_path = "/usr/local/src/affine2dtracking/results/error_matrix_added_latency.txt"
times_path = "/usr/local/src/affine2dtracking/results/failure_time_matrix_added_latency.txt"

times = np.loadtxt(times_path, delimiter=' ')

positions_s = [0,1,2,3]
speeds = ['240', '480', '720', '960']
positions_l = [0,1,2,3,4,5,6,7,8,9,10]
latencies = ['0', '5', '10', '15', '20', '25', '30', '35', '40', '45', '50']

input = np.loadtxt(errors_path, delimiter=' ')

matrix = np.matrix([input[0, :], input[1, :], input[2, :], input[3, :]])
fig, ax = plt.subplots(figsize=(10, 6))
im = plt.imshow(matrix, cmap='jet', interpolation='nearest')
plt.ylabel("speeds [pix/s]")
plt.xlabel("added latencies [ms]")
plt.title("Mean error [pix]")
plt.xticks(positions_l, latencies)
plt.yticks(positions_s, speeds)
cbar = fig.colorbar(im)
plt.clim(7,27) 

# text = ax.text(9, 0, round(times[0,9],2), ha="center", va="center", color="w")
# text = ax.text(10, 0, round(times[0,10],2), ha="center", va="center", color="w")

# text = ax.text(9, 1, round(times[1,9],2), ha="center", va="center", color="w")
# text = ax.text(10, 1, round(times[1,10],2), ha="center", va="center", color="w")

# text = ax.text(3, 2, round(times[2,3],2), ha="center", va="center", color="w")
# text = ax.text(4, 2, round(times[2,4],2), ha="center", va="center", color="w")
# text = ax.text(6, 2, round(times[2,6],2), ha="center", va="center", color="w")
# text = ax.text(7, 2, round(times[2,7],2), ha="center", va="center", color="w")
# text = ax.text(8, 2, round(times[2,8],2), ha="center", va="center", color="w")
# text = ax.text(9, 2, round(times[2,9],2), ha="center", va="center", color="w")
# text = ax.text(10, 2, round(times[2,10],2), ha="center", va="center", color="w")

# text = ax.text(3, 3, round(times[3,3],2), ha="center", va="center", color="w")
# text = ax.text(4, 3, round(times[3,4],2), ha="center", va="center", color="w")
# text = ax.text(5, 3, round(times[3,5],2), ha="center", va="center", color="w")
# text = ax.text(6, 3, round(times[3,6],2), ha="center", va="center", color="w")
# text = ax.text(7, 3, round(times[3,7],2), ha="center", va="center", color="w")
# text = ax.text(8, 3, round(times[3,8],2), ha="center", va="center", color="w")
# text = ax.text(9, 3, round(times[3,9],2), ha="center", va="center", color="w")
# text = ax.text(10, 3, round(times[3,10],2), ha="center", va="center", color="w")

matrix = np.matrix([times[0, :], times[1, :], times[2, :], times[3, :]])
fig, ax = plt.subplots(figsize=(10, 6))
im = plt.imshow(matrix, cmap='viridis', interpolation='nearest')
plt.ylabel("speeds [pix/s]")
plt.xlabel("added latencies [ms]")
plt.title("Time of failure [s]")
plt.xticks(positions_l, latencies)
plt.yticks(positions_s, speeds)
cbar = fig.colorbar(im)
plt.clim(1,7) 

plt.show()
