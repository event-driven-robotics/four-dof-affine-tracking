 
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

file_path = "star_combined_motions800hz.txt"

data = np.loadtxt(file_path, delimiter=" ", skiprows=3)[:,:11] # delete the first two lines

comp_time = data[1:, 9]
alg_speed = 1/comp_time
mean_alg_speed = np.mean(alg_speed)

plt.figure()

plt.plot(data[1:,0], alg_speed, color='g')
plt.axhline(y = mean_alg_speed, color = 'k', linestyle = '--')

plt.xlim(0,2)
plt.ylabel("Algorithm speed [Hz]")
plt.xlabel("Time [s]")

plt.show() 