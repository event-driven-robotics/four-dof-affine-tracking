 
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

ground_truth_trans_y = np.hstack((np.arange(1, 349, 1), np.arange(347, -349, -1), np.arange(-347, 1, 1)))
speeds_trans_y = [135,270,405,540,675,810]
gt_trans_y_list , t_trans_y_list= generate_ground_truth(speeds_trans_y, ground_truth_trans_y)

rotation_step = 0.3
ground_truth_rot = np.hstack((np.arange(rotation_step, 91.2+rotation_step,rotation_step), np.arange(90.9, -91.2+rotation_step,-rotation_step), np.arange(-90.9, 0+rotation_step, rotation_step)))
speeds_rot = [100,150,175,200,225,250]
gt_rot_list, t_rot_list = generate_ground_truth(speeds_rot, ground_truth_rot)


ground_truth_scale = compute_scale_vector()
speeds_scale = [100,200,300,400,600,800]
gt_scale_list, t_scale_list = generate_ground_truth(speeds_scale, ground_truth_scale)


arrow_path = "open-loop_results/arrow"
arrow_files = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), arrow_path)) if file.endswith('.txt')])
arrow_data_list = []

for f in tqdm(arrow_files, "Loading arrow shape data"):
    arrow_data = np.loadtxt(os.path.join(arrow_path, f), delimiter=" ", skiprows=2)[:,:15] # delete the first two lines
    arrow_data_list.append(arrow_data)

# for idx, x in enumerate(arrow_data_list):
    
#     if (idx>=0 and idx<6):
#         ax_gt_rot[idx].plot(arrow_data_list[idx][:, 0], arrow_data_list[idx][:, 5])

#     if (idx>=6 and idx<12):
#         ax_gt_scale[idx-6].plot(arrow_data_list[idx][:, 0], arrow_data_list[idx][:, 6])

#     if (idx>=12 and idx<18):
#         ax_gt_trans_x[idx-12].plot(arrow_data_list[idx][:, 0], arrow_data_list[idx][:, 3]) # 748 vs 562 and 212 vs 84

#     if (idx>=18 and idx<24):
#         ax_gt_trans_y[idx-18].plot(arrow_data_list[idx][:, 0], arrow_data_list[idx][:, 4])

arrow_error_trans_x_list, interp_trans_x = compute_error(t_trans_x_list,gt_trans_x_list,arrow_data_list,12,3,320)
arrow_error_trans_y_list, interp_trans_y = compute_error(t_trans_y_list,gt_trans_y_list,arrow_data_list,18,4,240)
arrow_error_rot_list, interp_rot = compute_error(t_rot_list,gt_rot_list,arrow_data_list,0,5,0)
arrow_error_scale_list, interp_scale = compute_error(t_scale_list, gt_scale_list, arrow_data_list, 6, 6, 1)

print(arrow_error_trans_x_list)
print(arrow_error_trans_y_list)
print(arrow_error_rot_list)
print(arrow_error_scale_list)

fig_error_time, ax_error_time = plt.subplots(2, 2)
ax_error_time[0,0].plot(t_trans_x_list[3], interp_trans_x[3], color='tab:blue',linewidth=1)
ax_error_time[0, 0].plot(t_trans_x_list[3], gt_trans_x_list[3], color='tab:blue', linestyle='dashed', linewidth=2)
ax_error_time[0, 1].plot(t_trans_y_list[3], interp_trans_y[3], color='tab:orange', linewidth=1)
ax_error_time[0, 1].plot(t_trans_y_list[3], gt_trans_y_list[3], color='tab:orange', linestyle='dashed', linewidth=2)
ax_error_time[1, 0].plot(t_rot_list[3], interp_rot[3], color='tab:green', linewidth=1)
ax_error_time[1, 0].plot(t_rot_list[3], gt_rot_list[3], color='tab:green', linestyle='dashed', linewidth=2)
ax_error_time[1, 1].plot(t_scale_list[3], interp_scale[3], color='tab:red', linewidth=1)
ax_error_time[1, 1].plot(t_scale_list[3], gt_scale_list[3], color='tab:red', linestyle='dashed', linewidth=2)

plt.setp(ax_error_time[-1, :], xlabel='Time [s]')
plt.setp(ax_error_time[1, 0], ylabel='Theta [deg]')
plt.setp(ax_error_time[1, 1], ylabel='Scale')
plt.setp(ax_error_time[0,0], ylabel='x [pix]')
plt.setp(ax_error_time[0,1], ylabel='y [pix]')

plt.show()

flag_path = "open-loop_results/flag"
flag_files = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), flag_path)) if file.endswith('.txt')])
flag_data_list = []

for f in tqdm(flag_files, "Loading flag shape data"):
    flag_data = np.loadtxt(os.path.join(flag_path, f), delimiter=" ", skiprows=2)[:,:15] # delete the first two lines
    flag_data_list.append(flag_data)

flag_error_trans_x_list, interp_trans_x = compute_error(t_trans_x_list,gt_trans_x_list,arrow_data_list,12,3,320)
flag_error_trans_y_list, interp_trans_y = compute_error(t_trans_y_list,gt_trans_y_list,arrow_data_list,18,4,240)
flag_error_rot_list, interp_rot = compute_error(t_rot_list,gt_rot_list,arrow_data_list,0,5,0)
flag_error_scale_list, interp_scale = compute_error(t_scale_list, gt_scale_list, arrow_data_list, 6, 6, 1)

print(flag_error_trans_x_list)
print(flag_error_trans_y_list)
print(flag_error_rot_list)
print(flag_error_scale_list)

shapes=["arrow", "flag"]
mean_rot_errors = [np.mean(arrow_error_rot_list), np.mean(flag_error_rot_list)]
n_shapes = np.arange(len(shapes))

fig_bar_plot, ax_bar_plot = plt.subplots()
ax_bar_plot.bar(n_shapes, mean_rot_errors, align='center', ecolor='black', alpha=0.5, capsize=10)
ax_bar_plot.set_ylabel('Rotation Error [deg]')
ax_bar_plot.set_xlabel('Shape')
ax_bar_plot.set_xticks(n_shapes)
ax_bar_plot.set_xticklabels(shapes)
ax_bar_plot.yaxis.grid(True)

# for i in range(6):
#     fig, ax1 = plt.subplots()

#     color = 'tab:red'
#     ax1.set_xlabel('time (s)')
#     ax1.set_ylabel('affine x', color=color)
#     ax1.plot(arrow_data_list[i+12][:,0], arrow_data_list[i+12][:,3], color=color)
#     ax1.tick_params(axis='y', labelcolor=color)

#     ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

#     color = 'tab:blue'
#     ax2.set_ylabel('gt x', color=color)  # we already handled the x-label with ax1
#     ax2.plot(t_trans_x_list[i], gt_trans_x_list[i], color=color)
#     ax2.tick_params(axis='y', labelcolor=color)

#     fig.tight_layout()  # otherwise the right y-label is slightly clipped


plt.show()

