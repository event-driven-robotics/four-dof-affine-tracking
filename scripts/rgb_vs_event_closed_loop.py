 
import os
import numpy as np
from tqdm import tqdm
import matplotlib 
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator
from scipy import interpolate
import csv
from itertools import zip_longest

params = {'xtick.labelsize': 24,
          'ytick.labelsize': 24,
          'font.size': 13,
          'figure.autolayout': True,  # similar to tight_layout
          'figure.figsize': [6.4, 4.8],  # ratio 1.6 is pleasant
          'axes.titlesize': 13,
          'axes.labelsize': 28,
          'lines.linewidth': 2,
          'lines.markersize': 4,
          'legend.fontsize': 20}

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)

data_recording_time = 10

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


# exp 1 rgb same gains as event camera different target speeds

def delete_useless_strings(path, file):
    delete_list = ['"[label: ""x""','size: 15\n', 'stride: 1]"', ',\n']

    with open(os.path.join(path, file)) as fin, open(os.path.join(path, "modified.csv"), "w") as fout:
        for line in fin:
            for word in delete_list:
                line = line.replace(word, "")
            fout.write(line)
        fout.close()

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

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

############################### event vs rgb same gain different target speeds ########################################################

file1 = 'ee_pose.csv'
file2 = 'modified.csv'
file3 = 'errors_and_velocities.csv'

files_list = []
directories_list = []

rgb_error_list = []
rgb_std_error_list = []

event_error_list = []
event_std_error_list = []

rgb_exp1_path = "/usr/local/src/affine2dtracking/bag_files/rgb_same_gain"
event_exp1_path = "/usr/local/src/affine2dtracking/bag_files/event_same_gain"


for f in os.listdir(event_exp1_path):
    files_list.append(f)

sorted_files = sorted(files_list)

for f in sorted_files:
    if ".txt" in f:
        event_exp1_data = np.loadtxt(os.path.join(event_exp1_path, f), delimiter=" ", skiprows=2)[:,:11] 
        delta_u = event_exp1_data[:,3]-320
        # plt.figure()
        # plt.plot(event_exp1_data[:,0], event_exp1_data[:,3], label='u')
        # plt.plot(event_exp1_data[:,0], event_exp1_data[:,4], label='v')
        # plt.plot(event_exp1_data[:,0], event_exp1_data[:,5], label='theta')
        # plt.plot(event_exp1_data[:,0], event_exp1_data[:,6], label='scale')
        # plt.xlabel('Time [s]')
        # plt.ylabel('State')
        # plt.title('Target speed ='+f)
        # plt.legend()
        # plt.show() 
    else: 

        if "ee_pose_1440ms" in f:
            event_ee_pose = np.loadtxt(os.path.join(event_exp1_path, f), delimiter=",", skiprows=1)[:,:8] 
    
            high_event_ee_pose = event_ee_pose[:,2]
            high_event_ee_time = event_ee_pose[:, 0]-event_ee_pose[0, 0]
        else:
            event_exp1_data = np.loadtxt(os.path.join(event_exp1_path, f), delimiter=",", skiprows=1)[:,:8] 
            timestamp = event_exp1_data[:,0]- event_exp1_data[0,0]
            delta_u = event_exp1_data[:,2]

            difference = delta_u[0:-2] - delta_u[1:-1] 
            starts = next(x for x, val in enumerate(difference) if val > 0)

            timestamp = timestamp[starts:-1]
            timestamp = timestamp[:]-timestamp[0]

            delta_u = delta_u[starts:-1]

            ends = next(x for x, val in enumerate(timestamp) if val > 10)

            timestamp = timestamp[:ends]
            delta_u = delta_u[:ends]            
            # plt.figure()
            # plt.plot(timestamp, delta_u+320, label='u')
            # plt.xlabel('Time [s]')
            # plt.ylabel('Delta theta [px]')
            # plt.title('Target speed ='+ f)
            # plt.legend()
            # plt.show()

    if not "ee_pose_1440ms" in f:
        mean_error = np.mean(abs(delta_u))
        std_error = np.std(abs(delta_u))

        event_error_list.append(mean_error)
        event_std_error_list.append(std_error)

        print(f + ", error = ", str(mean_error))

print("-------------------------------RGB-------------------------------------")

# ------------------------- RGB -------------------------------

for d in os.listdir(rgb_exp1_path):
    directories_list.append(d)

sorted_dir = sorted(directories_list)

for d in sorted_dir:
    rgb_exp1_ee_pose = np.loadtxt(os.path.join(rgb_exp1_path, d, file1), delimiter=",", skiprows=1)[:,:8] 
    
    rgb_exp1_ee_pose[:, 0] = rgb_exp1_ee_pose[:, 0]-rgb_exp1_ee_pose[0, 0]

    # fig, ax = plt.subplots(2,1)

    # ax[0].plot(rgb_exp1_ee_pose[:,0], rgb_exp1_ee_pose[:,1], label='x')
    # ax[0].plot(rgb_exp1_ee_pose[:,0], rgb_exp1_ee_pose[:,2], label='y')
    # ax[0].plot(rgb_exp1_ee_pose[:,0], rgb_exp1_ee_pose[:,3], label='z')
    # ax[1].plot(rgb_exp1_ee_pose[:,0], rgb_exp1_ee_pose[:,4], label='yaw')
    # ax[1].plot(rgb_exp1_ee_pose[:,0], rgb_exp1_ee_pose[:,5], label='pitch')
    # ax[1].plot(rgb_exp1_ee_pose[:,0], rgb_exp1_ee_pose[:,6], label='roll')
    
    # ax[0].set(xlabel="Time [s]", ylabel="Position [m]")
    # ax[1].set(xlabel="Time [s]", ylabel="Orientation [deg]")

    # ax[0].legend()
    # ax[1].legend()

    # plt.title(d)

    # plt.show()

    # plt.figure()
    # plt.plot(rgb_exp1_ee_pose[:,0], rgb_exp1_ee_pose[:,2], label='y ee')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Position [m]')
    # plt.title('Target speed ='+d)
    # plt.legend()

    rgb_exp1_data = np.loadtxt(os.path.join(rgb_exp1_path, d, file2), delimiter=",", skiprows=1)[:,:8] 
    timestamp = rgb_exp1_data[:,0]- rgb_exp1_data[0,0]
    delta_u = rgb_exp1_data[:,2]
    delta_v = rgb_exp1_data[:,3]
    delta_scale = rgb_exp1_data[:,4]
    delta_theta = rgb_exp1_data[:,5]

    if "240" in d:
        index = next(x for x, val in enumerate(timestamp) if val > 3)
        timestamp = timestamp[index:-1]
        delta_u = delta_u[index:-1]

        # low_rgb_ee_time = rgb_exp1_ee_pose[index:-1,0]- rgb_exp1_ee_pose[index,0]
        # low_rgb_ee_pose = rgb_exp1_ee_pose[index:-1,2]

        # plt.figure()
        # plt.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Position [m]')
        # plt.title('modified Target speed ='+d)
        # plt.legend()

    if "480" in d:

        coords = []
        fig, ax1 = plt.subplots(figsize=(15,8))
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        ax2 = ax1.twinx()
        plt.suptitle(d)

        ax1.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee', color = 'tab:green')
        ax2.plot(timestamp, delta_u, label='u', color = 'tab:purple')

        ax1.set(xlabel = 'Time [s]', ylabel="Position [m]")
        ax2.set(ylabel="Position [pix]")

        plt.show()

        cleaning_start_time = coords[0][0]

        index_start = find_nearest(rgb_exp1_ee_pose[:, 0], cleaning_start_time)
        index_start_du = find_nearest(timestamp, cleaning_start_time)

        low_rgb_ee_time = rgb_exp1_ee_pose[index_start:-1,0]- rgb_exp1_ee_pose[index_start,0]
        low_rgb_ee_pose = rgb_exp1_ee_pose[index_start:-1,2]

        # index = next(x for x, val in enumerate(timestamp) if val > 2)
        # timestamp = timestamp[index:-1]
        # delta_u = delta_u[index:-1]

        # low_rgb_ee_time = rgb_exp1_ee_pose[index:-1,0]- rgb_exp1_ee_pose[index,0]
        # low_rgb_ee_pose = rgb_exp1_ee_pose[index:-1,2]

        # plt.figure()
        # plt.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Position [m]')
        # plt.title('modified Target speed ='+d)
        # plt.legend()

    if "720" in d:
        index = next(x for x, val in enumerate(timestamp) if val > 1.5)
        timestamp = timestamp[index:-1]
        delta_u = delta_u[index:-1]

        # medium_rgb_ee_time = rgb_exp1_ee_pose[index:-1,0]- rgb_exp1_ee_pose[index,0]
        # medium_rgb_ee_pose = rgb_exp1_ee_pose[index:-1,2]

        plt.figure()
        plt.plot(timestamp, delta_u, label='delta u')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [pix]')
        plt.title('modified Target speed ='+d)
        plt.legend()
        plt.show()

    if "960" in d and not "960-2" in d:

        coords = []
        fig, ax1 = plt.subplots(figsize=(15,8))
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        ax2 = ax1.twinx()
        plt.suptitle(d)

        ax1.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee', color = 'tab:green')
        ax2.plot(timestamp, delta_u, label='u', color = 'tab:purple')

        ax1.set(xlabel = 'Time [s]', ylabel="Position [m]")
        ax2.set(ylabel="Position [pix]")

        plt.show()

        cleaning_start_time = coords[0][0]

        index_start = find_nearest(rgb_exp1_ee_pose[:, 0], cleaning_start_time)
        index_start_du = find_nearest(timestamp, cleaning_start_time)

        medium_rgb_ee_time = rgb_exp1_ee_pose[index_start:-1,0]- rgb_exp1_ee_pose[index_start,0]
        medium_rgb_ee_pose = rgb_exp1_ee_pose[index_start:-1,2]

        # index = next(x for x, val in enumerate(timestamp) if val > 3.8)
        # timestamp = timestamp[index:-1]
        # delta_u = delta_u[index:-1]

        # medium_rgb_ee_time = rgb_exp1_ee_pose[index:-1,0]- rgb_exp1_ee_pose[index,0]
        # medium_rgb_ee_pose = rgb_exp1_ee_pose[index:-1,2]

        # plt.figure()
        # plt.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Position [m]')
        # plt.title('modified Target speed ='+d)
        # plt.legend()

    
    if "1200" in d:
        index = next(x for x, val in enumerate(timestamp) if val > 2)
        timestamp = timestamp[index:-1]
        delta_u = delta_u[index:-1]

        # plt.figure()
        # plt.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Position [m]')
        # plt.title('modified Target speed ='+d)
        # plt.legend()

    if "1440-2" in d:

        coords = []
        fig, ax1 = plt.subplots(figsize=(15,8))
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        ax2 = ax1.twinx()
        plt.suptitle(d)

        ax1.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee', color = 'tab:green')
        ax2.plot(timestamp, delta_u, label='u', color = 'tab:purple')

        ax1.set(xlabel = 'Time [s]', ylabel="Position [m]")
        ax2.set(ylabel="Position [pix]")

        plt.show()

        cleaning_start_time = coords[0][0]

        index_start = find_nearest(rgb_exp1_ee_pose[:, 0], cleaning_start_time)
        index_start_du = find_nearest(timestamp, cleaning_start_time)

        high_rgb_ee_time = rgb_exp1_ee_pose[index_start:-1,0]- rgb_exp1_ee_pose[index_start,0]
        high_rgb_ee_pose = rgb_exp1_ee_pose[index_start:-1,2]

        # index = next(x for x, val in enumerate(timestamp) if val > 3.8)
        # timestamp = timestamp[index:-1]
        # delta_u = delta_u[index:-1]

        # high_rgb_ee_time = rgb_exp1_ee_pose[index:-1,0]- rgb_exp1_ee_pose[index,0]
        # high_rgb_ee_pose = rgb_exp1_ee_pose[index:-1,2]

        # plt.figure()
        # plt.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Position [m]')
        # plt.title('modified Target speed ='+d)
        # plt.legend()

    if "1440-2" in d:
        index = next(x for x, val in enumerate(timestamp) if val > 4.8)
        timestamp = timestamp[index:-1]
        delta_u = delta_u[index:-1]

        # high_rgb_ee_time = rgb_exp1_ee_pose[index:-1,0]-rgb_exp1_ee_pose[index,0]
        # high_rgb_ee_pose = rgb_exp1_ee_pose[index:-1,2]

        # plt.figure()
        # plt.plot(rgb_exp1_ee_pose[index:-1,0], rgb_exp1_ee_pose[index:-1,2], label='y ee')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Position [m]')
        # plt.title('modified Target speed ='+d)
        # plt.legend()

    # if the experiment did not fail cut to have some comparable duration
    if (timestamp[-1]>30):
        index_30 = next(x for x, val in enumerate(timestamp) if val > 30)
        timestamp = timestamp[0:index_30]
        delta_u = delta_u[0:index_30] 
        # print(d + " " + str(mean_error)+" pix")
        index_10 =  next(x for x, val in enumerate(timestamp) if val > 10)
        delta_u = delta_u[0:index_10] 
        mean_error = np.mean(abs(delta_u)) 
        std_error = np.std(abs(delta_u)) 
        print(d + ", error = ", str(mean_error))

    else:
        # if len(delta_u)>index_10:
        #     index_10 =  next(x for x, val in enumerate(timestamp) if val > 10)
        #     delta_u = delta_u[0:index_10] 

        difference = delta_u[0:-2] - delta_u[1:-1] 
        starts = next(x for x, val in enumerate(difference) if val > 0)

        timestamp = timestamp[starts:-1]
        timestamp = timestamp[:]-timestamp[0]

        delta_u = delta_u[starts:-1]

        mean_error = np.mean(abs(delta_u)) 
        std_error = np.std(abs(delta_u))

        print(d + ", failed, stopped at t = "+ str(timestamp[-1])+" s, mean error = "+ str(mean_error))


        # plt.figure()
        # plt.plot(timestamp, delta_u+320, label='u')
        # # plt.plot(timestamp, delta_v+240, label='v')
        # # plt.plot(timestamp, delta_theta, label='theta')
        # # plt.plot(timestamp, delta_scale+1, label='scale')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Delta theta [px]')
        # plt.title('Target speed ='+d)
        # plt.legend()
        # plt.show()

    # v_x_filtered, v_y_filtered, v_z_filtered, v_yaw_filtered,  Kv_y, Kv_x, Kv_z, Kv_yaw, filter, filter_z, filter_ang

    rgb_error_list.append(mean_error)
    rgb_std_error_list.append(std_error)

speeds = ['V1', 'V2', 'V3', 'V4', 'V5', 'V6']

rgb_mean_errors_target_speed = [rgb_error_list[0], rgb_error_list[1], rgb_error_list[2], rgb_error_list[3], rgb_error_list[5], rgb_error_list[7]]
event_mean_errors_target_speed = [event_error_list[0], event_error_list[1], event_error_list[2], event_error_list[3], event_error_list[7], event_error_list[6]]

diff_rgb_std = [rgb_error_list[0] - rgb_std_error_list[0], rgb_error_list[1] - rgb_std_error_list[1], rgb_error_list[2] - rgb_std_error_list[2]]
sum_rgb_std = [rgb_error_list[0] + rgb_std_error_list[0], rgb_error_list[1] + rgb_std_error_list[1], rgb_error_list[2] + rgb_std_error_list[2]]

diff_event_std = [event_error_list[0]-event_std_error_list[0], event_error_list[1]-event_std_error_list[1], event_error_list[2]-event_std_error_list[2], event_error_list[3]-event_std_error_list[3], event_error_list[7]-event_std_error_list[7], event_error_list[6]-event_std_error_list[6]]
sum_event_std = [event_error_list[0]+event_std_error_list[0], event_error_list[1]+event_std_error_list[1], event_error_list[2]+event_std_error_list[2], event_error_list[3]+event_std_error_list[3], event_error_list[7]+event_std_error_list[7], event_error_list[6]+event_std_error_list[6]]

rgb_mean_errors_target_speed_open_loop = [29.426525004815947, 29.884591665296284, 30.481688336522105, 29.558417963905644, 26.462313328247625, 28.83716602648601]
event_mean_errors_target_speed_open_loop = [4.8760896976188315, 6.252573943226012, 2.870520644215381, 2.437671783194352, 5.33733222205386, 3.941424425527976]

fig, ax1 = plt.subplots(figsize=(15, 8))
ax2 = ax1.twinx()
ax1.plot(speeds[0:3], rgb_mean_errors_target_speed[0:3], linestyle='-', color='tab:orange', label='real-sense (closed-loop)')
ax1.fill_between(speeds[0:3], diff_rgb_std, sum_rgb_std, alpha=0.2, color="tab:orange")
ax1.plot(speeds, event_mean_errors_target_speed, linestyle='-', label='event-camera (closed-loop)')
ax1.fill_between(speeds, diff_event_std, sum_event_std, alpha=0.2, color="tab:blue")
ax2.plot(speeds, event_mean_errors_target_speed_open_loop, color='tab:blue',linestyle='--', alpha = 0.5, label='event-camera (open-loop)')
ax2.plot(speeds, rgb_mean_errors_target_speed_open_loop, color='tab:orange', linestyle='--', alpha = 0.5, label='real-sense (open-loop)')
ax1.scatter(2, rgb_mean_errors_target_speed[2], s=100,marker='x',color='tab:orange',linewidths=2)
# plt.scatter(4,rgb_mean_errors_target_speed[2], s=100,marker='x',color='tab:orange',linewidths=2)
# plt.scatter(5,rgb_mean_errors_target_speed[2], s=100,marker='x',color='tab:orange',linewidths=2)
# plt.scatter(4,rgb_mean_errors_target_speed[4], s=100,marker='x',color='tab:orange',linewidths=2)
# plt.scatter(5,rgb_mean_errors_target_speed[5], s=100,marker='x',color='tab:orange',linewidths=2)
ax1.set_ylabel("Mean error [px]\n wrt reference")
ax2.set_ylabel("Mean error [px]\n wrt ground truth")
ax1.set_xlabel("Target speed")
ax1.set_ylim(0, 100)
ax2.set_ylim(0, 100)
# plt.title("Kp = 0.005")
ax1.legend( loc='upper left')
ax2.legend( loc='upper right')
plt.show()

############################### RGB same target speed different gains #######################################################

# FILTER 0.005

rgb_exp2_path = "/usr/local/src/affine2dtracking/bag_files/rgb_diff_gains"

directories_list_2 = []
rgb_mean_error_gains = []

for d in os.listdir(rgb_exp2_path):
    directories_list_2.append(d)

sorted_dir_2 = sorted(directories_list_2)

for d in sorted_dir_2:
    rgb_exp2_ee_pose = np.loadtxt(os.path.join(rgb_exp2_path, d, file1), delimiter=",", skiprows=1)[:,:8] 
    rgb_exp2_ee_pose[:, 0] = rgb_exp2_ee_pose[:, 0]-rgb_exp2_ee_pose[0, 0]

    rgb_exp2_data = np.loadtxt(os.path.join(rgb_exp2_path, d, file2), delimiter=",", skiprows=1)[:,:8] 
    timestamp = rgb_exp2_data[:,0]- rgb_exp2_data[0,0]
    delta_u = rgb_exp2_data[:,2]
    # delta_v = rgb_exp1_data[:,3]
    # delta_scale = rgb_exp1_data[:,4]
    # delta_theta = rgb_exp1_data[:,5]

    # plt.figure()
    # plt.plot(rgb_exp2_ee_pose[:,0], rgb_exp2_ee_pose[:,2], label='y ee')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Position [m]')
    # plt.title('Target speed and gain value ='+d)
    # plt.legend()

    # plt.figure()
    # plt.plot(timestamp, delta_u+320, label='u')
    # # plt.plot(timestamp, delta_v+240, label='v')
    # # plt.plot(timestamp, delta_theta, label='theta')
    # # plt.plot(timestamp, delta_scale+1, label='scale')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Delta u [px]')
    # plt.title('Target speed and gain value ='+ d)
    # plt.legend()

    if "0.008" in d:
        index = next(x for x, val in enumerate(timestamp) if val > 2)
        timestamp = timestamp[index:-1]
        delta_u = delta_u[index:-1]

    if "0.012" in d and not ("0.012-2" in d):
        index = next(x for x, val in enumerate(timestamp) if val > 3.5)
        timestamp = timestamp[index:-1]
        delta_u = delta_u[index:-1]

    if "0.012" in d and ("0.012-2" in d):
        index = next(x for x, val in enumerate(timestamp) if val > 2)
        timestamp = timestamp[index:-1]
        delta_u = delta_u[index:-1]

    if (index_10< len(delta_u)):
        index_10 = next(x for x, val in enumerate(timestamp) if val > 10)
        timestamp = timestamp[0:index_10]
        delta_u = delta_u[0:index_10]

    mean_error = np.mean(abs(delta_u))

    rgb_mean_error_gains.append(mean_error)

    print(d + " --- error = " + str(mean_error))

# FILTER 0.002

rgb_exp2_path_2 = "/usr/local/src/affine2dtracking/bag_files/rgb-camera-gains"

directories_list_2 = []
rgb_mean_error_gains_2 = []
rgb_std_error_gains_2 = []

# for d in os.listdir(rgb_exp2_path_2):
#     directories_list_2.append(d)

# sorted_dir_2 = sorted(directories_list_2)

# for d in sorted_dir_2:

#     coords = []

#     rgb_exp2_ee_pose_2 = np.loadtxt(os.path.join(rgb_exp2_path_2, d, file1), delimiter=",", skiprows=1)[:,:8] 
#     rgb_exp2_ee_pose_2[:, 0] = rgb_exp2_ee_pose_2[:, 0]-rgb_exp2_ee_pose_2[0, 0]

#     rgb_exp2_data_2 = np.loadtxt(os.path.join(rgb_exp2_path_2, d, file2), delimiter=",", skiprows=1)[:,:8] 
#     timestamp = rgb_exp2_data_2[:,0]- rgb_exp2_data_2[0,0]
#     delta_u = rgb_exp2_data_2[:,2]
#     fig, ax = plt.subplots(3,1)
#     cid = fig.canvas.mpl_connect('button_press_event', onclick)

#     plt.suptitle(d)

#     ax[0].plot(rgb_exp2_ee_pose_2[:, 0], rgb_exp2_ee_pose_2[:, 2], label='y ee')
#     ax[1].plot(timestamp, delta_u+320, label='u')
#     ax[2].plot(timestamp, delta_u, label='delta u')

#     ax[0].set(ylabel="Position [m]")
#     ax[1].set(ylabel="Position [pix]")
#     ax[2].set(xlabel="Time [s]", ylabel="Error [pix]")

#     plt.show()

#     cleaning_start_time = coords[0][0]
#     cleaning_end_time = coords[1][0]

#     index_start = find_nearest(rgb_exp2_ee_pose_2[:, 0], cleaning_start_time)
#     index_end = find_nearest(rgb_exp2_ee_pose_2[:, 0], cleaning_end_time)

#     index_start_du = find_nearest(timestamp, cleaning_start_time)
#     index_end_du = find_nearest(timestamp, cleaning_end_time)

#     new_ee_time = rgb_exp2_ee_pose_2[index_start:index_end, 0]
#     new_ee_time = new_ee_time-new_ee_time[0]
#     new_ee_pose = rgb_exp2_ee_pose_2[index_start:index_end, 2]
#     new_delta_u = delta_u[index_start_du:index_end_du]
#     mergeddata = [new_ee_time, new_ee_pose, new_delta_u]
#     new_ee_zip = zip_longest(*mergeddata, fillvalue = '')

#     file_name = 'cleaned_data.csv'
#     with open(os.path.join(rgb_exp2_path_2, d,file_name), 'w') as myfile:
#         wr = csv.writer(myfile)
#         wr.writerows(new_ee_zip)
#     myfile.close()

for d in os.listdir(rgb_exp2_path_2):
    directories_list_2.append(d)

sorted_dir_3 = sorted(directories_list_2)

for d in sorted_dir_3:
    rgb_gains_cleaned = np.loadtxt(os.path.join(rgb_exp2_path_2, d, "cleaned_data.csv"), delimiter=",")[:,:4] 
    
    if (rgb_gains_cleaned[-1,0]>10):
        index = next(x for x, val in enumerate(rgb_gains_cleaned[:,0]) if val > 10)
        time_10s = rgb_gains_cleaned[:index,0] 
        ee_pose_10s = rgb_gains_cleaned[:index,1] 
        delta_u_10s = rgb_gains_cleaned[:index,2] 

        mean_error = np.mean(abs(delta_u_10s))
        std_error = np.std(abs(delta_u_10s))

        rgb_mean_error_gains_2.append(mean_error)
        rgb_std_error_gains_2.append(std_error)

        print(d + " --- error = " + str(mean_error))


# EVENT CAMERA DIFF GAINS (CLEANING)

event_exp2_path = "/usr/local/src/affine2dtracking/bag_files/event-camera-diff-gains"

directories_list_3 = []
event_mean_error_gains = []
event_std_error_gains = []

# for d in os.listdir(event_exp2_path):
#     directories_list_3.append(d)

# sorted_dir_3 = sorted(directories_list_3)

# for d in sorted_dir_3:
#     coords = []

    # delete_useless_strings(os.path.join(event_exp2_path, d), file3)

    # event_exp2_ee_pose = np.loadtxt(os.path.join(event_exp2_path, d, file1), delimiter=",", skiprows=1)[:,:11] 
    # event_exp2_ee_pose[:, 0] = event_exp2_ee_pose[:, 0]-event_exp2_ee_pose[0, 0]
    # event_exp2_data = np.loadtxt(os.path.join(event_exp2_path, d, file2), delimiter=",", skiprows=1)[:,:16] 
    # timestamp = event_exp2_data[:,0]- event_exp2_data[0,0]
    # delta_u = event_exp2_data[:,2]

    # fig, ax = plt.subplots(3,1)
    # cid = fig.canvas.mpl_connect('button_press_event', onclick)

    # plt.suptitle(d + "-> gain = "+ str(event_exp2_data[0,11])+ ", filter value = "+  str(event_exp2_data[0,15]))

    # ax[0].plot(event_exp2_ee_pose[:, 0], event_exp2_ee_pose[:, 2], label='y ee')
    # ax[1].plot(timestamp, delta_u+320, label='u')
    # ax[2].plot(timestamp, delta_u, label='delta u')

    # ax[0].set(ylabel="Position [m]")
    # ax[1].set(ylabel="Position [pix]")
    # ax[2].set(xlabel="Time [s]", ylabel="Error [pix]")

    # plt.show()

    # cleaning_start_time = coords[0][0]
    # cleaning_end_time = coords[1][0]

    # index_start = find_nearest(event_exp2_ee_pose[:, 0], cleaning_start_time)
    # index_end = find_nearest(event_exp2_ee_pose[:, 0], cleaning_end_time)

    # index_start_du = find_nearest(timestamp, cleaning_start_time)
    # index_end_du = find_nearest(timestamp, cleaning_end_time)

    # new_ee_time = event_exp2_ee_pose[index_start:index_end, 0]
    # new_ee_time = new_ee_time-new_ee_time[0]
    # new_ee_pose = event_exp2_ee_pose[index_start:index_end, 2]
    # new_delta_u = delta_u[index_start_du:index_end_du]
    # mergeddata = [new_ee_time, new_ee_pose, new_delta_u]
    # new_ee_zip = zip_longest(*mergeddata, fillvalue = '')

    # file_name = 'cleaned_data.csv'
    # with open(os.path.join(event_exp2_path, d,file_name), 'w') as myfile:
    #     wr = csv.writer(myfile)
    #     wr.writerows(new_ee_zip)
    # myfile.close()

for d in os.listdir(event_exp2_path):
    directories_list_3.append(d)

sorted_dir_3 = sorted(directories_list_3)

for d in sorted_dir_3:
    event_gains_cleaned = np.loadtxt(os.path.join(event_exp2_path, d, "cleaned_data.csv"), delimiter=",")[:,:4] 
    
    if (event_gains_cleaned[-1,0]>10):
        index = next(x for x, val in enumerate(event_gains_cleaned[:,0]) if val > 10)
        time_10s = event_gains_cleaned[:index,0] 
        ee_pose_10s = event_gains_cleaned[:index,1] 
        delta_u_10s = event_gains_cleaned[:index,2] 

        mean_error = np.mean(abs(delta_u_10s))
        std_error = np.std(abs(delta_u_10s))

        event_mean_error_gains.append(mean_error)
        event_std_error_gains.append(std_error)

        print(d + " --- error = " + str(mean_error))

        if "960" in d:
            medium_event_ee_time = time_10s
            medium_event_ee_pose = ee_pose_10s

        if "480_0.002_0.005" in d:
            low_event_ee_time = time_10s
            low_event_ee_pose = ee_pose_10s
    
    # fig, ax = plt.subplots(2,1)
    # ax[0].plot(event_gains_cleaned[:,0], event_gains_cleaned[:,1])
    # ax[1].plot(event_gains_cleaned[:,0], event_gains_cleaned[:,2])
    # plt.suptitle(d)
    # ax[0].set(xlabel="Position [m]")
    # ax[1].set(xlabel="Time [s]", ylabel="Error [pix]")
    # plt.show()


gains = ['0.002', '0.004', '0.006', '0.008', '0.010', '0.012', '0.014', '0.018', '0.022', '0.026', '0.030', '0.038']

rgb_mean_errors_diff_gains = [rgb_mean_error_gains[2], rgb_mean_error_gains[3], rgb_mean_error_gains[5], rgb_mean_error_gains[6], rgb_mean_error_gains[7], rgb_mean_error_gains[8], rgb_mean_error_gains[9], rgb_mean_error_gains[9], rgb_mean_error_gains[9], rgb_mean_error_gains[9], rgb_mean_error_gains[9], rgb_mean_error_gains[9]]
rgb_mean_errors_diff_gains_2 = [rgb_mean_error_gains_2[0], rgb_mean_error_gains_2[1], rgb_mean_error_gains_2[2], rgb_mean_error_gains_2[3], rgb_mean_error_gains_2[4], rgb_mean_error_gains_2[4], rgb_mean_error_gains[4], rgb_mean_error_gains[4], rgb_mean_error_gains[4], rgb_mean_error_gains[4], rgb_mean_error_gains[4], rgb_mean_error_gains[4]]

event_mean_errors_diff_gains = [event_mean_error_gains[0], event_mean_error_gains[1], event_mean_error_gains[3], event_mean_error_gains[4], event_mean_error_gains[5], event_mean_error_gains[6], event_mean_error_gains[7], event_mean_error_gains[8], event_mean_error_gains[9], event_mean_error_gains[10], event_mean_error_gains[11], event_mean_error_gains[12]]
event_mean_errors_diff_gains_2 = [event_mean_error_gains[14], event_mean_error_gains[14], event_mean_error_gains[14], event_mean_error_gains[15], event_mean_error_gains[16], event_mean_error_gains[17], event_mean_error_gains[18], event_mean_error_gains[18], event_mean_error_gains[18], event_mean_error_gains[18], event_mean_error_gains[18], event_mean_error_gains[18]]

diff_rgb_std_gains_2 = [rgb_mean_error_gains_2[0]-rgb_std_error_gains_2[0], rgb_mean_error_gains_2[1]-rgb_std_error_gains_2[1], rgb_mean_error_gains_2[2]-rgb_std_error_gains_2[2], rgb_mean_error_gains_2[3]-rgb_std_error_gains_2[3], rgb_mean_error_gains_2[4]-rgb_std_error_gains_2[4]]
sum_rgb_std_gains_2 = [rgb_mean_error_gains_2[0]+rgb_std_error_gains_2[0], rgb_mean_error_gains_2[1]+rgb_std_error_gains_2[1], rgb_mean_error_gains_2[2]+rgb_std_error_gains_2[2], rgb_mean_error_gains_2[3]+rgb_std_error_gains_2[3], rgb_mean_error_gains_2[4]+rgb_std_error_gains_2[4]]

diff_event_std_gains = [event_mean_error_gains[0]-event_std_error_gains[0], event_mean_error_gains[1]-event_std_error_gains[1], event_mean_error_gains[3]-event_std_error_gains[3], event_mean_error_gains[4]-event_std_error_gains[4], event_mean_error_gains[5]-event_std_error_gains[5], event_mean_error_gains[6]-event_std_error_gains[6], event_mean_error_gains[7]-event_std_error_gains[7], event_mean_error_gains[8]-event_std_error_gains[8], event_mean_error_gains[9]-event_std_error_gains[9], event_mean_error_gains[10]-event_std_error_gains[10], event_mean_error_gains[11]-event_std_error_gains[11], event_mean_error_gains[12]-event_std_error_gains[12]]
sum_event_std_gains = [event_mean_error_gains[0]+event_std_error_gains[0], event_mean_error_gains[1]+event_std_error_gains[1], event_mean_error_gains[3]+event_std_error_gains[3], event_mean_error_gains[4]+event_std_error_gains[4], event_mean_error_gains[5]+event_std_error_gains[5], event_mean_error_gains[6]+event_std_error_gains[6], event_mean_error_gains[7]+event_std_error_gains[7], event_mean_error_gains[8]+event_std_error_gains[8], event_mean_error_gains[9]+event_std_error_gains[9], event_mean_error_gains[10]+event_std_error_gains[10], event_mean_error_gains[11]+event_std_error_gains[11], event_mean_error_gains[12]+event_std_error_gains[12]]

plt.figure(figsize=(15, 8))
# plt.plot(gains[0:3], rgb_mean_errors_diff_gains[0:3], linestyle='-.', color='tab:orange', label='real-sense filter = 0.005', alpha = 0.5)
# plt.plot(gains[3:11], rgb_mean_errors_diff_gains[3:11], linestyle='-.', color='white')
plt.plot(gains[0:5], rgb_mean_errors_diff_gains_2[0:5], linestyle='-', color='tab:orange', label='real-sense')
plt.fill_between(gains[0:5], diff_rgb_std_gains_2, sum_rgb_std_gains_2, alpha=0.2, color="tab:orange")
plt.plot(gains[0:11], event_mean_errors_diff_gains[0:11], linestyle='-', color='tab:blue', label='event-camera')
plt.fill_between(gains[0:11], diff_event_std_gains[0:11], sum_event_std_gains[0:11], alpha=0.2, color="tab:blue")
# plt.plot(gains[0:2], event_mean_errors_diff_gains_2[0:2], linestyle='-.', color='white')
# plt.plot(gains[2:7], event_mean_errors_diff_gains_2[2:7], linestyle='-.', color='tab:blue', label='event camera filter = 0.005', alpha = 0.5)
# plt.scatter(3,rgb_mean_errors_diff_gains[3], s=100,marker='x',color='tab:orange',linewidths=2)
plt.scatter(4,rgb_mean_errors_diff_gains_2[4], s=100,marker='x',color='tab:orange',linewidths=2)
# plt.scatter(6,rgb_mean_errors_diff_gains_2[4], s=100,marker='x',color='tab:orange',linewidths=2)

# plt.scatter(4,rgb_mean_errors_diff_gains[4], s=100,marker='x',color='tab:orange',linewidths=2)
# plt.scatter(5,rgb_mean_errors_diff_gains[5], s=100,marker='x',color='tab:orange',linewidths=2)
plt.ylabel("Mean error [px]")
plt.xlabel("Control gain")
# plt.title("Target speed = 480 pix/s,\nControl filter = 0.002")
plt.legend()
plt.show()

speeds_trans_x = [240,480,720,960,1200,1440]
ground_truth_trans_x = np.hstack((np.arange(1, 768+1, 1), np.arange(767, -769, -1), np.arange(-767, 1, 1)))
gt_trans_x_list_pix, t_trans_x_list_pix = generate_ground_truth(speeds_trans_x, ground_truth_trans_x)

H = [[3.42086, -0.0618227, -56.3823], 
     [0.193321, 3.37744, -287.184], 
     [8.97966e-05, 8.93452e-05, 1]]

u_list = []
v_list = []
for i in range(6):
    u_gt = []
    v_gt = []
    for j in range(len(gt_trans_x_list_pix[i])):
        coord_camera_plane_gt = np.array([gt_trans_x_list_pix[i][j],gt_trans_x_list_pix[i][j], 1])
        result_gt = np.matmul(np.linalg.inv(H), coord_camera_plane_gt)

        u_gt.append(result_gt[0]/result_gt[2])
        v_gt.append(result_gt[1]/result_gt[2])

    u_list.append(u_gt)
    v_list.append(v_gt)

fig, ax1 = plt.subplots(3,1, figsize=(15, 8))
ax2 = ax1[0].twinx()
ax3 = ax1[1].twinx()
ax4 = ax1[2].twinx()
# print(high_rgb_ee_pose[0])

# high_rgb_ee_pose = high_rgb_ee_pose + high_event_ee_pose[0]

ax1[0].plot(low_rgb_ee_time, low_rgb_ee_pose, label = 'real-sense',  ls='--', color='tab:orange')
ax1[0].plot(low_event_ee_time, low_event_ee_pose-low_event_ee_pose[0], label = 'event camera', color='tab:blue')
ax2.plot(t_trans_x_list_pix[1], -gt_trans_x_list_pix[1], label = 'ground truth', color = 'tab:green')
ax1[1].plot(medium_rgb_ee_time, medium_rgb_ee_pose, label = 'real-sense',  ls='--', color='tab:orange')
ax1[1].plot(medium_event_ee_time, medium_event_ee_pose-medium_event_ee_pose[0], label = 'event camera', color='tab:blue')
ax3.plot(t_trans_x_list_pix[3], -gt_trans_x_list_pix[3], label = 'ground truth', color = 'tab:green')
ax1[2].plot(high_rgb_ee_time, high_rgb_ee_pose, label = 'real-sense', ls='--', color='tab:orange')
ax1[2].plot(high_event_ee_time, high_event_ee_pose-high_event_ee_pose[0], label = 'event camera', color='tab:blue')
ax4.plot(t_trans_x_list_pix[5], -gt_trans_x_list_pix[5], label = 'ground truth', color = 'tab:green')
ax1[0].set_title("Low target speed", fontsize = 20.0)
ax1[1].set_title("Medium target speed", fontsize = 20.0)
ax1[2].set_title("High target speed", fontsize = 20.0)
ax1[2].set_xlabel('Time [s]')
# ax1[0].set_ylabel('Position [m]')
# ax2.set_ylabel('Position [px]')
ax1[1].set_ylabel('End-effector position [m]')
ax3.set_ylabel('Target position [px]')
# ax1[2].set_ylabel('Position [m]')
# ax4.set_ylabel('Position [px]')
ax1[0].set_xlim([0, 10])
ax1[1].set_xlim([0, 10])
ax1[2].set_xlim([0, 10])
ax1[0].legend(bbox_to_anchor=(0.12, 1.25, 0, 0.2), loc='lower left', ncol=2)
ax2.legend(bbox_to_anchor=(0.61, 1.25, 0, 0.2), loc='lower left', ncol=1)

# ax[0].set_legend(loc = 'upper right')
plt.show()

# plt.figure(figsize=(10, 6))
# plt.plot(high_rgb_ee_time, high_rgb_ee_pose, label = 'real-sense', ls='--', color='tab:orange')
# plt.plot(high_event_ee_time, high_event_ee_pose, label = 'event camera', color='tab:blue')
# plt.xlabel('Time [s]')
# plt.ylabel('Position [m]')

# plt.xlim([0, 10])
# plt.legend(loc = 'upper right')
# # ax[0].set_legend(loc = 'upper right')
# plt.show()

