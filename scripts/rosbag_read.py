 
import os
import numpy as np
from tqdm import tqdm
import matplotlib 
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator
from scipy import interpolate
import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea

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

bag_path = "bagfiles/latency"
bag_files = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), bag_path)) if file.endswith('.bag')])
bag_data_list = []

for bag_name in tqdm(bag_files, "Loading bag data"):
    print(bag_name)
    b = bagreader(os.path.join(bag_path,bag_name))
    tee = "/ee_pose"
    ttracking = "/errors_and_velocities"
    end_effector = b.message_by_topic(tee)
    tracking = b.message_by_topic(ttracking)
    ee_pose_data = pd.read_csv(end_effector)
    tracking_data = pd.read_csv(tracking)

    fig, ax = bagpy.create_fig(6)
    ax[0].scatter(x = 'Time', y = 'position.x', data  = ee_pose_data, s= 1, label = 'x end effector')
    ax[1].scatter(x = 'Time', y = 'position.y', data  = ee_pose_data, s= 1, label ='y end effector')
    ax[2].scatter(x = 'Time', y = 'position.z', data  = ee_pose_data, s= 1, label = 'z end effector')
    ax[3].scatter(x = 'Time', y = 'orientation.x', data  = ee_pose_data, s= 1, label = 'yaw end effector')
    ax[4].scatter(x = 'Time', y = 'data_0', data  = tracking_data, s= 1, label = 'delta u', color='red')
    ax[5].scatter(x = 'Time', y = 'data_0', data  = tracking_data, s= 1, label = 'v_y_filtered', color='red')

    for axis in ax:
        axis.legend()
        axis.set_xlabel('Time')

    plt.show()