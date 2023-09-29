 
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

bag_path = "bagfiles_31-03/1920"
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

    index_start = next(x for x, val in enumerate(tracking_data['data_0']) if val > 0)
    ee_pose_data = ee_pose_data[index_start:-1]
    tracking_data = tracking_data[index_start:-1]

    array1 = np.array(tracking_data[30:-1]['data_0'])
    array2 = np.array(tracking_data[0:-31]['data_0'])

    variation_delta_u = array1 - array2

    count_succ_zeros = 0
    for idx, value in enumerate(variation_delta_u):
        if value == 0:
            count_succ_zeros += 1 
        else:
            count_succ_zeros = 0
        if (count_succ_zeros>300):
            index_end = idx-300
            break 

    ee_pose_data = ee_pose_data[0:index_end]
    tracking_data = tracking_data[0:index_end]

    tracking_data['Time'] = np.array(tracking_data['Time'])-np.array(tracking_data['Time'])[0]

    # fig, ax = bagpy.create_fig(6)
    # ax[0].scatter(x = 'Time', y = 'position.x', data  = ee_pose_data, s= 1, label = 'x end effector')
    # ax[1].scatter(x = 'Time', y = 'position.y', data  = ee_pose_data, s= 1, label ='y end effector')
    # ax[2].scatter(x = 'Time', y = 'position.z', data  = ee_pose_data, s= 1, label = 'z end effector')
    # ax[3].scatter(x = 'Time', y = 'orientation.x', data  = ee_pose_data, s= 1, label = 'yaw end effector')
    # ax[4].scatter(x = 'Time', y = 'data_0', data  = tracking_data, s= 1, label = 'delta u', color='red')
    # ax[5].scatter(x = 'Time', y = 'data_5', data  = tracking_data, s= 1, label = 'v_y_filtered', color='green')

    # print("Kv_y=",tracking_data['data_8'])
    # print("filter=",tracking_data['data_12'])

    # for axis in ax:
    #     axis.legend()
    #     axis.set_xlabel('Time')

    # plt.title(bag_name)

    # fig, ax1 = plt.subplots()
    # ax2 = ax1.twinx()

    # ax1.plot(tracking_data['Time'], tracking_data['data_0'], color='r', label = 'error x')
    # ax1.plot(tracking_data['Time'], tracking_data['data_1'], color='g', label = 'error y')
    # ax1.plot(tracking_data['Time'], tracking_data['data_2'], color='b', label = 'error z')
    # ax2.plot(tracking_data['Time'], tracking_data['data_3'], color='m', label = 'error yaw')
    # ax2.axhline(y = 0, color = 'k', linestyle = '-')

    # ax1.set_xlabel('Time [s]')
    # ax1.set_ylabel('Position error \n [pixels]')
    # ax2.set_ylabel('Rotation error \n [rad]')

    # plt.show()

    fig, axs = plt.subplots(4)
    axs[0].plot(tracking_data['Time'], tracking_data['data_0'], color='r')
    axs[0].axhline(y = 0, color = 'r', linestyle = '--')
    axs[0].set_ylim(-100,100)

    axs[1].plot(tracking_data['Time'], tracking_data['data_1'], color='g')
    axs[1].axhline(y = 0, color = 'g', linestyle = '--')
    axs[1].set_ylim(-100,100)

    axs[2].plot(tracking_data['Time'], tracking_data['data_3'], color='b')
    axs[2].axhline(y = 0, color = 'b', linestyle = '--')
    axs[2].set_ylim(-100,100)

    axs[3].plot(tracking_data['Time'], tracking_data['data_2'], color='m')
    axs[3].axhline(y = 0, color = 'm', linestyle = '--')

    axs[0].set_ylabel('error x \n [pix]')
    axs[1].set_ylabel('error y \n [pix]')
    axs[2].set_ylabel('error z \n [pix]')
    axs[3].set_ylabel('error yaw \n [rad]')
    axs[3].set_xlabel('Time [s]')

    plt.show()