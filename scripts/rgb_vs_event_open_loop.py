 
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

w = 1920 
h = 1080
data_recording_time = 10

def compute_error(t_list, gt_list, rgb_list, motion_index, column_state, reference, shape):

    error_list = []

    plot_qualitative = False

    if plot_qualitative:
        fig_error, ax_error = plt.subplots(2,3)   
    for i in range(len(gt_list)):

        f = interpolate.interp1d(rgb_list[i][:,0], (rgb_list[motion_index+i][:,column_state]-reference), kind='nearest',fill_value="extrapolate")

        times = t_list[i]

        y_ = f(times)
        
        diff = abs(y_ - gt_list[i])
        error_list.append(diff)

        if plot_qualitative:
            ax_error[i//3, i%3].plot(times,diff, label = 'error')
            ax_error[i//3, i%3].plot(times,y_, label = 'tracked')
            ax_error[i//3, i%3].plot(times,gt_list[i], label = 'gt')
            ax_error[i//3, i%3].set_title("speed #"+str(i))
            ax_error[i//3, i%3].legend(loc="upper right")
            
        if plot_qualitative:
            plt.setp(ax_error[-1], xlabel='Time [s]')
            if (column_state == 3):
                plt.setp(ax_error[:, 0], ylabel='x [cm]')
            if (column_state == 4):
                plt.setp(ax_error[:], ylabel='y [cm]')
            if (column_state == 5):
                plt.setp(ax_error[:], ylabel=r'$\theta$[deg]')
            if (column_state == 6):
                plt.setp(ax_error[:], ylabel='scale')
            plt.suptitle(shape+" translation x")
            plt.legend()

        
    if plot_qualitative:
        plt.show()

    return error_list

def generate_ground_truth(speed_array, gt_array):

    gt_list = []
    t_list = []
    for i, s in enumerate(speed_array):

        video_duration = len(gt_array)/s
        t = np.arange(0, video_duration, video_duration/len(gt_array))
        ratio = int(data_recording_time/video_duration) + 1
        gt_enlarged = np.tile(gt_array, ratio)
        t_enlarged = np.arange(0, video_duration*ratio, video_duration/len(gt_array))
        t_cut = t_enlarged[t_enlarged<data_recording_time]
        gt_cut = gt_enlarged[0:len(t_cut)]
        gt_list.append(gt_cut)
        t_list.append(t_cut)
        # ax_plot[i].plot(t_cut, gt_cut)

    return gt_list, t_list

# 29.150789785474558
# 29.884591665296284
# 31.03002166985544
# 28.504265282190865
# 26.462313328247625
# 27.59092730177253

# 29.426525004815947                                                                                               
# 29.884591665296284
# 30.481688336522105
# 29.558417963905644
# 26.462313328247625
# 28.83716602648601

# [4.8760896976188315, 6.252573943226012, 2.870520644215381, 2.437671783194352, 5.33733222205386, 3.941424425527976]

speeds = ['V1', 'V2', 'V3', 'V4', 'V5', 'V6']

event_mean_errors_target_speed = [0.26337317, 0.1214185,  0.1383508, 0.68607668, 0.70300897, 0.71994126]
rgb_mean_errors_target_speed = [1.772681435743793, 1.8266990359626447,  2.0078972198398763, 1.8049797713071984, 1.6275385393192279, 1.7459249503366177]

plt.figure(figsize=(10, 6))
plt.plot(speeds, event_mean_errors_target_speed, label='event-camera')
plt.plot(speeds, rgb_mean_errors_target_speed, label='real-sense')

plt.ylabel("Mean error [cm]")
plt.xlabel("Target speed")
# plt.title("Kp = 0.005")
plt.legend()
plt.show()
