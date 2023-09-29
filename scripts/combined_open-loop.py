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

def load_ground_truth(filename):
    groud_truth_combined_motions = np.loadtxt(filename, delimiter=" ")
    groud_truth_combined_motions[:,0] = groud_truth_combined_motions[:,0]+1920/2
    groud_truth_combined_motions[:,1] = groud_truth_combined_motions[:,1]+1080/2
    return groud_truth_combined_motions


def generate_ground_truth(speed, gt_array):

    gt_list = []
    t_list = []
    gt_rows, gt_cols = gt_array.shape
    for i in range(gt_cols):

        video_duration = gt_rows/speed
        t = np.arange(0, video_duration, video_duration/gt_rows)
        ratio = int(data_recording_time/video_duration) + 1
        gt_enlarged = np.tile(gt_array[:,i], ratio)
        t_enlarged = np.arange(0, video_duration*ratio, video_duration/gt_rows)
        t_cut = t_enlarged[t_enlarged<data_recording_time]
        gt_cut = gt_enlarged[0:len(t_cut)]
        gt_list.append(gt_cut)
        t_list.append(t_cut)
        # ax_plot[i].plot(t_cut, gt_cut)

    return gt_list, t_list

data_recording_time = 10

ground_truth_combined = load_ground_truth("gt_combined_motions.txt")
# speed_combined = 800
speed_combined = 100
gt_comb_list, t_comb_list = generate_ground_truth(speed_combined, ground_truth_combined)

tracked_shape_path = "open-loop_results/arrow/arrow_4dof_100hz.txt"

shape_data = np.loadtxt(tracked_shape_path, delimiter=" ", skiprows=3)[:,:15] # delete the first two lines

# index_time_cut = next(x for x, val in enumerate(shape_data[:,0]) if val > 4)

# H = np.array([[3.34885, 0.139952, -146.819],[-0.0350819, 3.35032, -226.383],[-1.21935e-05, 9.92637e-05, 1]])
#H = np.array([[3.33477, 0.171014, -95.2546], [-0.0587216, 3.14961, -155.954], [3.57572e-05, 6.0639e-05, 1]]) 
# H = np.array([[3.32553, 0.156037, -98.6833], [-0.0943547, 3.24691, -182.526], [1.58806e-05, 6.63458e-05, 1]]) 
# H = np.array([[3.39687, -0.0709793, -62.0461], [0.195592, 3.31389, -278.965], [7.71912e-05, 6.48431e-05, 1]]) # arrow 100 hz
H = np.array([[3.32006, -0.109672, -50.3293] , [0.188382, 3.29114, -286.429], [4.44017e-05, 3.67536e-05, 1 ]])

u_monitor_plane = []
v_monitor_plane = []
for idx, x in enumerate(shape_data[:,3]):
    coord_camera_plane = np.array([shape_data[idx,3],shape_data[idx,4], 1])
    result = np.matmul(H, coord_camera_plane)

    u_monitor_plane.append(result[0]/result[2])
    v_monitor_plane.append(result[1]/result[2])

# scale_monitor_plane = shape_data[:,6]*2.88

fig, axs = plt.subplots(2,2)

# trans_x
axs[0,0].plot(t_comb_list[0], np.divide(gt_comb_list[0],1920)*32.51, label = 'gt', color='tab:blue')
axs[0,0].plot(shape_data[:,0], np.divide(u_monitor_plane,1920)*32.51, linestyle ='dashed', label = 'tracked', color='tab:orange')

# trans_y
axs[0,1].plot(t_comb_list[1], np.divide(gt_comb_list[1],1080)*18.29, label = 'gt', color='tab:blue')
axs[0,1].plot(shape_data[:,0], np.divide(v_monitor_plane,1080)*18.29, linestyle ='dashed', label = 'tracked', color='tab:orange')

# rotation
axs[1,0].plot(t_comb_list[3], gt_comb_list[3], label = 'gt', color='tab:blue')
axs[1,0].plot(shape_data[:,0], shape_data[:,5], linestyle ='dashed', label = 'tracked', color='tab:orange')

# scale
axs[1,1].plot(t_comb_list[2], gt_comb_list[2], label = 'gt', color='tab:blue')
axs[1,1].plot(shape_data[:,0], shape_data[:,6], linestyle ='dashed', label = 'tracked', color='tab:orange')

axs[0,0].set_ylabel('x \n [cm]')
axs[0,1].set_ylabel('y \n [cm]')
axs[1,0].set_ylabel('$\Theta$ \n [rad]')
axs[1,1].set_ylabel('sc')
axs[0,0].set_xlabel('Time [s]')
axs[0,1].set_xlabel('Time [s]')
axs[1,0].set_xlabel('Time [s]')
axs[1,1].set_xlabel('Time [s]')

axs[0,0].legend()
axs[1,0].legend()
axs[0,1].legend()
axs[1,1].legend()

plt.show()




# normalization
# fig, axs = plt.subplots(2,2)
# max_gt = np.max(gt_comb_list[0])
# max_tracking = np.max(shape_data[:,3])
# # t_comb_list[0]=t_comb_list[0][0:index_time_cut]
# # gt_comb_list[0] = t_comb_list[0][0:index_time_cut]

# axs[0,0].plot(t_comb_list[0], gt_comb_list[0]/max_gt, label = 'tracked')
# axs[0,0].plot(shape_data[:,0], shape_data[:,3]/max_tracking, linestyle ='dashed', label = 'gt')
# axs[0,0].set_xlim(0,4)

# max_gt = np.max(gt_comb_list[1])
# max_tracking = np.max(shape_data[:,4])

# axs[0,1].plot(t_comb_list[1], gt_comb_list[1]/max_gt, label = 'tracked')
# axs[0,1].plot(shape_data[:,0], shape_data[:,4]/max_tracking, linestyle ='dashed', label = 'gt')
# axs[0,1].set_xlim(0,4)

# max_gt = np.max(gt_comb_list[2])
# max_tracking = np.max(shape_data[:,6])

# axs[1,0].plot(t_comb_list[2], gt_comb_list[2]/max_gt, label = 'tracked')
# axs[1,0].plot(shape_data[:,0], shape_data[:,6]/max_tracking, linestyle ='dashed', label = 'gt')
# axs[1,0].set_xlim(0,4)

# max_gt = np.max(gt_comb_list[3])
# max_tracking = np.max(shape_data[:,5])

# axs[1,1].plot(t_comb_list[3], gt_comb_list[3]/max_gt, label = 'tracked')
# axs[1,1].plot(shape_data[:,0], shape_data[:,5]/max_tracking, linestyle ='dashed', label = 'gt')
# axs[1,1].set_xlim(0,4)

# axs[0,0].set_ylabel('u \n [pix]')
# axs[0,1].set_ylabel('v \n [pix]')
# axs[1,0].set_ylabel('sc \n [pix]')
# axs[1,1].set_ylabel('$\Theta$ \n [rad]')
# axs[0,0].set_xlabel('Time [s]')
# axs[0,1].set_xlabel('Time [s]')
# axs[1,0].set_xlabel('Time [s]')
# axs[1,1].set_xlabel('Time [s]')

# plt.legend()

# plt.show()

