import os
import numpy as np
from tqdm import tqdm
from scipy import interpolate
import matplotlib 
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator

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

def compute_scale_vector():

    up_scale_limit = 1.5 
    up_scale_step = 1.001

    down_scale_limit = 0.5
    down_scale_step = 0.999

    scale = 1 

    scale_list = []

    while scale <  up_scale_limit:
        scale *= up_scale_step
        scale_list.append(scale)

    while scale > down_scale_limit:
        scale *= down_scale_step
        scale_list.append(scale)

    while not(scale > 1 and scale <  up_scale_step):
        scale *= up_scale_step   
        scale_list.append(scale)

    scale_array = np.array(scale_list)

    return scale_array

def compute_error(t_list, gt_list, shape_list, motion_index, column_state, reference):

    mean_error = []
    new_x_list = []

    for i in range(6):
        f = interpolate.interp1d(shape_list[motion_index+i][:,0], (shape_list[motion_index+i][:,column_state]-reference), kind='nearest',fill_value="extrapolate")
        y_ = f(t_list[i])

        from sklearn.linear_model import LinearRegression
        model = LinearRegression().fit(np.array(y_).reshape((-1, 1)), np.array(gt_list[i]))
        intercept_b = model.intercept_
        coeff_ang_m = model.coef_

        new_x = coeff_ang_m*(y_)+intercept_b

        new_x_list.append(new_x)

        diff = abs(gt_list[i]-new_x)

        mean_error.append(np.mean(diff))

        fig_error = plt.figure()
        plt.plot(t_list[i],diff)
        plt.plot(t_list[i],new_x)
        plt.plot(t_list[i],gt_list[i])

    return mean_error, new_x_list

w = 1920 
h = 1080
data_recording_time = 10

ground_truth_trans_x = np.hstack((np.arange(1, 768+1, 1), np.arange(767, -769, -1), np.arange(-767, 1, 1)))
speeds_trans_x = [240,480,720,960,1200,1440]
gt_trans_x_list, t_trans_x_list = generate_ground_truth(speeds_trans_x, ground_truth_trans_x)

t_sampled = np.arange(0, 10, 0.001)

import csv

for i in range(6):
    f = interpolate.interp1d(t_trans_x_list[i], gt_trans_x_list[i], kind='nearest',fill_value="extrapolate")
    gt_sampled = f(t_sampled)
    with open('gt_trans_x_'+str(speeds_trans_x[i])+'Hz.csv', 'w') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerows(zip(t_sampled,gt_sampled))