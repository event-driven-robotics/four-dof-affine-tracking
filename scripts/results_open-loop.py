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

color_blue = '#1A589F'
color_purple = '#742C81' 
color_yellow = '#FFA62B' 

w = 1920 
h = 1080
data_recording_time = 10

def load_ground_truth(filename):
    groud_truth_combined_motions = np.loadtxt(filename, delimiter=" ")
    groud_truth_combined_motions[:,0] = groud_truth_combined_motions[:,0]+1920/2
    groud_truth_combined_motions[:,1] = groud_truth_combined_motions[:,1]+1080/2
    return groud_truth_combined_motions

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


def generate_ground_truth_combined(speed, gt_array):

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

def sameFreqDiffGT(t_array, gt_array):

    t_interpolated = []
    gt_interpolated = []
    t_freq_1000hz = np.arange(0,10, 0.001)

    f_x = interpolate.interp1d(t_array[0], gt_array[0], kind='nearest',fill_value="extrapolate")
    x_interpolated = f_x(t_freq_1000hz)
    t_interpolated.append(t_freq_1000hz)
    gt_interpolated.append(x_interpolated)

    f_y = interpolate.interp1d(t_array[0], gt_array[1], kind='nearest',fill_value="extrapolate")
    y_interpolated = f_y(t_freq_1000hz)
    t_interpolated.append(t_freq_1000hz)
    gt_interpolated.append(y_interpolated)

    f_rot = interpolate.interp1d(t_array[0], gt_array[3], kind='nearest',fill_value="extrapolate")
    rot_interpolated = f_rot(t_freq_1000hz)
    t_interpolated.append(t_freq_1000hz)
    gt_interpolated.append(rot_interpolated)

    f_sc = interpolate.interp1d(t_array[0], gt_array[2], kind='nearest',fill_value="extrapolate")
    sc_interpolated = f_sc(t_freq_1000hz)
    t_interpolated.append(t_freq_1000hz)
    gt_interpolated.append(sc_interpolated)

    return t_interpolated, gt_interpolated


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

def compute_error(t_list, gt_list, shape_list, motion_index, column_state, reference, shape):

    error_list = []

    plot_qualitative = False

    if plot_qualitative:
        fig_error, ax_error = plt.subplots(2,3)   
    for i in range(len(gt_list)):

        if (shape == 'rectangle' and (motion_index == 18 or motion_index == 24)):
            error_list = []
        elif (shape == 'square' and (motion_index == 18)):
            error_list = []
        elif (shape == 'hexagon' and (motion_index == 6)):
            error_list = []
        else:
            f = interpolate.interp1d(shape_list[motion_index+i][:,0], (shape_list[motion_index+i][:,column_state]-reference), kind='nearest',fill_value="extrapolate")

            if (motion_index >= 0 and motion_index < 6):
                times = t_list[i][0]
            else:
                times = t_list[i]

            y_ = f(times)

            if (motion_index == 6 and column_state == 5):
                y_ = -y_
            if (motion_index == 12 and column_state == 6):
                y_ = abs(y_-1)+1
            if (motion_index == 18 and i == 0 and shape != "arrow"):
                y_ = -y_
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

            if (motion_index == 0):
                plt.suptitle(shape+" 4dof")
            if (motion_index == 6):
                plt.suptitle(shape+" rotate")
            if (motion_index == 12):
                plt.suptitle(shape+" scale")
            if (motion_index == 18):
                plt.suptitle(shape+" translation x")
            if (motion_index == 24):
                plt.suptitle(shape+" translation y")

            plt.legend()

        
    if plot_qualitative:
        plt.show()

    return error_list

def computeEuclideanDistance(errors_x_list, errors_y_list):
    distances = []
    for i in range(len(errors_x_list)):
        distances.append(np.sqrt(np.multiply(np.array(errors_x_list[i]),np.array(errors_x_list[i]))+np.multiply(np.array(errors_y_list[i]),np.array(errors_y_list[i]))))

    return distances

ground_truth_trans_x = np.hstack((np.arange(1, 768+1, 1), np.arange(767, -769, -1), np.arange(-767, 1, 1)))
ground_truth_trans_x = np.divide(ground_truth_trans_x, 1920)*32.51
speeds_trans_x = [240,480,720,960,1200,1440]
gt_trans_x_list, t_trans_x_list = generate_ground_truth(speeds_trans_x, ground_truth_trans_x)
ground_truth_trans_x_y = []
ground_truth_trans_x_rot = []
ground_truth_trans_x_sc = []
for idx, value in enumerate(gt_trans_x_list):
    ground_truth_trans_x_y.append((18.29/2)*np.ones(len(value)))
    ground_truth_trans_x_rot.append(np.zeros(len(value)))
    ground_truth_trans_x_sc.append(np.ones(len(value)))

ground_truth_trans_y = np.hstack((np.arange(1, 349, 1), np.arange(347, -349, -1), np.arange(-347, 1, 1)))
ground_truth_trans_y = np.divide(ground_truth_trans_y, 1080)*18.29
speeds_trans_y = [135,270,405,540,675,810]
gt_trans_y_list , t_trans_y_list= generate_ground_truth(speeds_trans_y, ground_truth_trans_y)
ground_truth_trans_y_x = []
ground_truth_trans_y_rot = []
ground_truth_trans_y_sc = []
for idx, value in enumerate(gt_trans_y_list):
    ground_truth_trans_y_x.append((32.51/2)*np.ones(len(value)))
    ground_truth_trans_y_rot.append(np.zeros(len(value)))
    ground_truth_trans_y_sc.append(np.ones(len(value)))

rotation_step = 0.3
ground_truth_rot = np.hstack((np.arange(rotation_step, 91.2+rotation_step,rotation_step), np.arange(90.9, -91.2+rotation_step,-rotation_step), np.arange(-90.9, 0+rotation_step, rotation_step)))
speeds_rot = [100,150,175,200,225,250]
gt_rot_list, t_rot_list = generate_ground_truth(speeds_rot, ground_truth_rot)
ground_truth_rot_x = []
ground_truth_rot_y = []
ground_truth_rot_sc = []
for idx, value in enumerate(gt_rot_list):
    ground_truth_rot_x.append((32.51/2)*np.ones(len(value)))
    ground_truth_rot_y.append((18.29/2)*np.ones(len(value)))
    ground_truth_rot_sc.append(np.ones(len(value)))

ground_truth_scale = compute_scale_vector()
speeds_scale = [100,200,300,400,600,800]
gt_scale_list, t_scale_list = generate_ground_truth(speeds_scale, ground_truth_scale)
ground_truth_scale_x = []
ground_truth_scale_rot = []
ground_truth_scale_y = []
for idx, value in enumerate(gt_scale_list):
    ground_truth_scale_x.append((32.51/2)*np.ones(len(value)))
    ground_truth_scale_y.append((18.29/2)*np.ones(len(value)))
    ground_truth_scale_rot.append(np.zeros(len(value)))

ground_truth_combined = load_ground_truth("gt_combined_motions.txt")
speed_combined = [100,200,300,400,600,800]

gt_comb_speeds = []
t_comb_speeds = []

for count, value in enumerate(speed_combined):
    gt_comb_list, t_comb_list = generate_ground_truth_combined(value, ground_truth_combined)

    t_interp, gt_interp = sameFreqDiffGT(t_comb_list, gt_comb_list)
    currentFileName = "gt_comb_" + str(value) + ".csv"
    with open(currentFileName, 'w') as f:
        for a, b, c, d, e in zip(t_interp[0], gt_interp[0], gt_interp[1], gt_interp[2], gt_interp[3]):
            print("%0.5f,%0.2f,%0.2f,%0.2f,%0.2f" % (a, b, c, d, e), file = f)
    # currentFile = open(currentFileName, 'r+')
    # np.savetxt(currentFileName, np.hstack((np.array(t_comb_list), np.array(gt_comb_list))), fmt='%f')
    # currentFile.close()
    gt_comb_speeds.append(gt_comb_list)
    t_comb_speeds.append(t_comb_list)

x_gt_comb_list = []
y_gt_comb_list = []
rot_gt_comb_list = []
sc_gt_comb_list = []
for speed in range(6):  
    x_gt_comb_list.append(np.divide(gt_comb_speeds[speed][0], 1920)*32.51)
    y_gt_comb_list.append(np.divide(gt_comb_speeds[speed][1], 1080)*18.29)
    rot_gt_comb_list.append(gt_comb_speeds[speed][3])
    sc_gt_comb_list.append(gt_comb_speeds[speed][2])

error_x_all_shapes = []
error_y_all_shapes = []
error_dist_all_shapes = []
error_rot_all_shapes = []
error_sc_all_shapes = []

shape_names = ['arrow', 'flag', 'oval', 'pacman', 'puzzle', 'square', 'star', 'triangle']

for idx_shape, shape in enumerate(shape_names):
    shape_path = "open-loop_results/"+shape
    shape_files = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), shape_path)) if file.endswith('.txt')])
    shape_data_list = []

    H_list = []

    for f in tqdm(shape_files, "Loading " + shape + " data"):

        lines = []
        with open(os.path.join("open-loop_results/arrow/homography/homography.txt")) as linefile:
            for row in range(3):
                line = linefile.readline().strip('\n')
                line_list = list(map(float, line.split(' ')))
                lines.append(line_list)
                
            H_list.append(np.array(lines))

        shape_data = np.loadtxt(os.path.join(shape_path, f), delimiter=" ", skiprows=3)[:,:15] # delete the first six lines
        shape_data_list.append(shape_data)

    for idx_global, x_global in enumerate(shape_data_list):

        u_monitor_plane = []
        v_monitor_plane = []
        x_cm = []
        y_cm = []
        rot_comb = []
        sc_comb = []

        for idx, x in enumerate(x_global):
            coord_camera_plane = np.array([x_global[idx][3],x_global[idx][4], 1])
            result = np.matmul(H_list[idx_global], coord_camera_plane)

            u_monitor_plane.append(result[0]/result[2])
            v_monitor_plane.append(result[1]/result[2])

            x_cm.append(np.divide(result[0]/result[2],1920)*32.51)
            y_cm.append(np.divide(result[1]/result[2],1080)*18.29)
            rot_comb.append(shape_data_list[idx_global][idx][5])
            sc_comb.append(shape_data_list[idx_global][idx][6])

            shape_data_list[idx_global][idx][3] = np.divide(result[0]/result[2],1920)*32.51
            shape_data_list[idx_global][idx][4] = np.divide(result[1]/result[2],1080)*18.29

    error_x_4dof_list = compute_error(t_comb_speeds, x_gt_comb_list, shape_data_list, 0, 3, 0, shape)
    error_y_4dof_list = compute_error(t_comb_speeds, y_gt_comb_list, shape_data_list, 0, 4, 0, shape)
    error_rot_4dof_list = compute_error(t_comb_speeds, rot_gt_comb_list, shape_data_list, 0, 5, 0, shape)
    error_sc_4dof_list = compute_error(t_comb_speeds, sc_gt_comb_list, shape_data_list, 0, 6, 0, shape)
    error_dist_4dof_list = computeEuclideanDistance(error_x_4dof_list, error_y_4dof_list)

    error_x_4dof_all_speeds = np.concatenate((error_x_4dof_list[0], error_x_4dof_list[1], error_x_4dof_list[2], error_x_4dof_list[3], error_x_4dof_list[4], error_x_4dof_list[5]))
    error_y_4dof_all_speeds = np.concatenate((error_y_4dof_list[0], error_y_4dof_list[1], error_y_4dof_list[2], error_y_4dof_list[3], error_y_4dof_list[4], error_y_4dof_list[5]))
    error_dist_4dof_all_speeds = np.concatenate((error_dist_4dof_list[0], error_dist_4dof_list[1], error_dist_4dof_list[2], error_dist_4dof_list[3], error_dist_4dof_list[4], error_dist_4dof_list[5]))
    error_rot_4dof_all_speeds = np.concatenate((error_rot_4dof_list[0], error_rot_4dof_list[1], error_rot_4dof_list[2], error_rot_4dof_list[3], error_rot_4dof_list[4], error_rot_4dof_list[5]))
    error_sc_4dof_all_speeds = np.concatenate((error_sc_4dof_list[0], error_sc_4dof_list[1], error_sc_4dof_list[2], error_sc_4dof_list[3], error_sc_4dof_list[4], error_sc_4dof_list[5]))


    error_x_trans_x_list = compute_error(t_trans_x_list,gt_trans_x_list,shape_data_list,18,3,32.51/2, shape)
    error_y_trans_x_list = compute_error(t_trans_x_list,ground_truth_trans_x_y,shape_data_list,18,4,0, shape)
    error_rot_trans_x_list = compute_error(t_trans_x_list,ground_truth_trans_x_rot,shape_data_list,18,5,0, shape)
    error_sc_trans_x_list = compute_error(t_trans_x_list,ground_truth_trans_x_sc,shape_data_list,18,6,0, shape)
    error_dist_trans_x_list = computeEuclideanDistance(error_x_trans_x_list, error_y_trans_x_list)

    error_x_trans_x_all_speeds = []
    error_y_trans_x_all_speeds = []
    error_dist_trans_x_all_speeds = []
    error_rot_trans_x_all_speeds = []
    error_sc_trans_x_all_speeds = []

    if error_x_trans_x_list != []:
        error_x_trans_x_all_speeds = np.concatenate((error_x_trans_x_list[0], error_x_trans_x_list[1], error_x_trans_x_list[2], error_x_trans_x_list[3], error_x_trans_x_list[4], error_x_trans_x_list[5]))
    if error_y_trans_x_list != []:
        error_y_trans_x_all_speeds = np.concatenate((error_y_trans_x_list[0], error_y_trans_x_list[1], error_y_trans_x_list[2], error_y_trans_x_list[3], error_y_trans_x_list[4], error_y_trans_x_list[5]))
    if error_x_trans_x_list != [] and error_y_trans_x_list != []:
        error_dist_trans_x_all_speeds = np.concatenate((error_dist_trans_x_list[0], error_dist_trans_x_list[1], error_dist_trans_x_list[2], error_dist_trans_x_list[3], error_dist_trans_x_list[4], error_dist_trans_x_list[5]))
    if error_rot_trans_x_list != []:
        error_rot_trans_x_all_speeds = np.concatenate((error_rot_trans_x_list[0], error_rot_trans_x_list[1], error_rot_trans_x_list[2], error_rot_trans_x_list[3], error_rot_trans_x_list[4], error_rot_trans_x_list[5]))
    if error_sc_trans_x_list != []:
        error_sc_trans_x_all_speeds = np.concatenate((error_sc_trans_x_list[0], error_sc_trans_x_list[1], error_sc_trans_x_list[2], error_sc_trans_x_list[3], error_sc_trans_x_list[4], error_sc_trans_x_list[5]))


    error_y_trans_y_list = compute_error(t_trans_y_list,gt_trans_y_list,shape_data_list,24,4,18.29/2, shape)
    error_x_trans_y_list = compute_error(t_trans_y_list,ground_truth_trans_y_x,shape_data_list,24,3,0, shape)
    error_rot_trans_y_list = compute_error(t_trans_y_list,ground_truth_trans_y_rot,shape_data_list,24,5,0, shape)
    error_sc_trans_y_list = compute_error(t_trans_y_list,ground_truth_trans_y_sc,shape_data_list,24,6,0, shape)
    error_dist_trans_y_list = computeEuclideanDistance(error_x_trans_y_list, error_y_trans_y_list)
    
    error_x_trans_y_all_speeds = []
    error_y_trans_y_all_speeds = []
    error_dist_trans_y_all_speeds = []
    error_rot_trans_y_all_speeds = []
    error_sc_trans_y_all_speeds = []

    if error_x_trans_y_list != []:
        error_x_trans_y_all_speeds = np.concatenate((error_x_trans_y_list[0], error_x_trans_y_list[1], error_x_trans_y_list[2], error_x_trans_y_list[3], error_x_trans_y_list[4], error_x_trans_y_list[5]))
    if error_y_trans_y_list != []:
        error_y_trans_y_all_speeds = np.concatenate((error_y_trans_y_list[0], error_y_trans_y_list[1], error_y_trans_y_list[2], error_y_trans_y_list[3], error_y_trans_y_list[4], error_y_trans_y_list[5]))
    if error_x_trans_y_list != [] and error_y_trans_y_list != []:
        error_dist_trans_y_all_speeds = np.concatenate((error_dist_trans_y_list[0], error_dist_trans_y_list[1], error_dist_trans_y_list[2], error_dist_trans_y_list[3], error_dist_trans_y_list[4], error_dist_trans_y_list[5]))
    if error_rot_trans_y_list != []:
        error_rot_trans_y_all_speeds = np.concatenate((error_rot_trans_y_list[0], error_rot_trans_y_list[1], error_rot_trans_y_list[2], error_rot_trans_y_list[3], error_rot_trans_y_list[4], error_rot_trans_y_list[5]))
    if error_sc_trans_y_list != []:
        error_sc_trans_y_all_speeds = np.concatenate((error_sc_trans_y_list[0], error_sc_trans_y_list[1], error_sc_trans_y_list[2], error_sc_trans_y_list[3], error_sc_trans_y_list[4], error_sc_trans_y_list[5]))


    error_rot_rot_list = compute_error(t_rot_list,gt_rot_list,shape_data_list,6,5,0, shape)
    error_x_rot_list = compute_error(t_rot_list,ground_truth_rot_x,shape_data_list,6,3,0, shape)
    error_y_rot_list = compute_error(t_rot_list,ground_truth_rot_y,shape_data_list,6,4,0, shape)
    error_sc_rot_list = compute_error(t_rot_list,ground_truth_rot_sc,shape_data_list,6,6,0, shape)
    error_dist_rot_list = computeEuclideanDistance(error_x_rot_list, error_y_rot_list)

    error_x_rot_all_speeds = []
    error_y_rot_all_speeds = []
    error_rot_rot_all_speeds = []
    error_sc_rot_all_speeds = []
    error_dist_rot_all_speeds = []

    if error_x_rot_list != []:
        error_x_rot_all_speeds = np.concatenate((error_x_rot_list[0], error_x_rot_list[1], error_x_rot_list[2], error_x_rot_list[3], error_x_rot_list[4], error_x_rot_list[5]))   
    if error_y_rot_list != []:
        error_y_rot_all_speeds = np.concatenate((error_y_rot_list[0], error_y_rot_list[1], error_y_rot_list[2], error_y_rot_list[3], error_y_rot_list[4], error_y_rot_list[5]))
    if error_x_rot_list != [] and error_y_rot_list != []:
        error_dist_rot_all_speeds = np.concatenate((error_dist_rot_list[0], error_dist_rot_list[1], error_dist_rot_list[2], error_dist_rot_list[3], error_dist_rot_list[4], error_dist_rot_list[5]))
    if error_rot_rot_list != []:
        error_rot_rot_all_speeds = np.concatenate((error_rot_rot_list[0], error_rot_rot_list[1], error_rot_rot_list[2], error_rot_rot_list[3], error_rot_rot_list[4], error_rot_rot_list[5]))
    if error_sc_rot_list != []:
        error_sc_rot_all_speeds = np.concatenate((error_sc_rot_list[0], error_sc_rot_list[1], error_sc_rot_list[2], error_sc_rot_list[3], error_sc_rot_list[4], error_sc_rot_list[5]))

    error_sc_scale_list = compute_error(t_scale_list, gt_scale_list, shape_data_list, 12, 6, 0, shape)
    error_x_scale_list = compute_error(t_scale_list, ground_truth_scale_x, shape_data_list, 12, 3, 0, shape)
    error_y_scale_list = compute_error(t_scale_list, ground_truth_scale_y, shape_data_list, 12, 4, 0, shape)
    error_rot_scale_list = compute_error(t_scale_list, ground_truth_scale_rot, shape_data_list, 12, 5, 0, shape)
    error_dist_scale_list = computeEuclideanDistance(error_x_scale_list, error_y_scale_list)

    error_x_sc_all_speeds = np.concatenate((error_x_scale_list[0], error_x_scale_list[1], error_x_scale_list[2], error_x_scale_list[3], error_x_scale_list[4], error_x_scale_list[5]))
    error_y_sc_all_speeds = np.concatenate((error_y_scale_list[0], error_y_scale_list[1], error_y_scale_list[2], error_y_scale_list[3], error_y_scale_list[4], error_y_scale_list[5]))
    error_dist_sc_all_speeds = np.concatenate((error_dist_scale_list[0], error_dist_scale_list[1], error_dist_scale_list[2], error_dist_scale_list[3], error_dist_scale_list[4], error_dist_scale_list[5]))
    error_rot_sc_all_speeds = np.concatenate((error_rot_scale_list[0], error_rot_scale_list[1], error_rot_scale_list[2], error_rot_scale_list[3], error_rot_scale_list[4], error_rot_scale_list[5]))
    error_sc_sc_all_speeds = np.concatenate((error_sc_scale_list[0], error_sc_scale_list[1], error_sc_scale_list[2], error_sc_scale_list[3], error_sc_scale_list[4], error_sc_scale_list[5]))



    error_x_all_motions_and_speeds = np.concatenate((error_x_4dof_all_speeds, error_x_trans_x_all_speeds, error_x_trans_y_all_speeds, error_x_rot_all_speeds, error_x_sc_all_speeds))
    error_y_all_motions_and_speeds = np.concatenate((error_y_4dof_all_speeds, error_y_trans_x_all_speeds, error_y_trans_y_all_speeds, error_y_rot_all_speeds, error_y_sc_all_speeds))
    error_dist_all_motions_and_speeds = np.concatenate((error_dist_4dof_all_speeds, error_dist_trans_x_all_speeds, error_dist_trans_y_all_speeds, error_dist_rot_all_speeds, error_dist_sc_all_speeds))
    error_rot_all_motions_and_speeds = np.concatenate((error_rot_4dof_all_speeds, error_rot_trans_x_all_speeds, error_rot_trans_y_all_speeds, error_rot_rot_all_speeds, error_rot_sc_all_speeds))
    error_sc_all_motions_and_speeds = np.concatenate((error_sc_4dof_all_speeds, error_sc_trans_x_all_speeds, error_sc_trans_y_all_speeds, error_sc_rot_all_speeds, error_sc_sc_all_speeds))

    error_x_all_shapes.append(error_x_all_motions_and_speeds)
    error_y_all_shapes.append(error_y_all_motions_and_speeds)
    error_dist_all_shapes.append(error_dist_all_motions_and_speeds)
    error_rot_all_shapes.append(error_rot_all_motions_and_speeds)
    error_sc_all_shapes.append(error_sc_all_motions_and_speeds)

    n_speeds = len(speed_combined)

    vel_labels = ['V6','V5','V4','V3','V2', 'V1']
    ticks=[0,1,2,3,4,5]
    medianprops = dict(color='black')

    vec_x_pos_4dof=[np.array(error_x_4dof_list[5]), np.array(error_x_4dof_list[4]), np.array(error_x_4dof_list[3]), np.array(error_x_4dof_list[2]), np.array(error_x_4dof_list[1]), np.array(error_x_4dof_list[0])]
    vec_y_pos_4dof=[np.array(error_y_4dof_list[5]), np.array(error_y_4dof_list[4]), np.array(error_y_4dof_list[3]), np.array(error_y_4dof_list[2]), np.array(error_y_4dof_list[1]), np.array(error_y_4dof_list[0])]
    vec_dist_4dof=[np.array(error_dist_4dof_list[5]), np.array(error_dist_4dof_list[4]), np.array(error_dist_4dof_list[3]), np.array(error_dist_4dof_list[2]), np.array(error_dist_4dof_list[1]), np.array(error_dist_4dof_list[0])]
    vec_rot_4dof=[np.array(error_rot_4dof_list[5]), np.array(error_rot_4dof_list[4]), np.array(error_rot_4dof_list[3]), np.array(error_rot_4dof_list[2]), np.array(error_rot_4dof_list[1]), np.array(error_rot_4dof_list[0])]
    vec_sc_4dof=[np.array(error_sc_4dof_list[5]), np.array(error_sc_4dof_list[4]), np.array(error_sc_4dof_list[3]), np.array(error_sc_4dof_list[2]), np.array(error_sc_4dof_list[1]), np.array(error_sc_4dof_list[0])]

    fig_speeds, ax1 = plt.subplots(1,3)
    ax1[0].set_xlabel('Position error [cm]', color='k')
    # ax1[1].set_xlabel('y position error [cm]', color='k')
    ax1[1].set_xlabel('Rotation error [deg]', color='k')
    ax1[2].set_xlabel('Scale error', color='k')
    res1 = ax1[0].boxplot(vec_dist_4dof, labels=vel_labels, vert=False, showfliers=False,
                        patch_artist=True, medianprops=medianprops)
    res2 = ax1[1].boxplot(vec_rot_4dof, vert=False, showfliers=False,
                    patch_artist=True, medianprops=medianprops)
    res3 = ax1[2].boxplot(vec_sc_4dof, vert=False, showfliers=False,
                    patch_artist=True, medianprops=medianprops)
    for element in ['boxes', 'whiskers', 'fliers', 'means', 'medians', 'caps']:
        plt.setp(res1[element], color='k')
        plt.setp(res2[element], color='k')
        plt.setp(res3[element], color='k')
    ax1[1].set_yticklabels([])
    ax1[1].set_yticks([])
    ax1[2].set_yticklabels([])
    ax1[2].set_yticks([])
    ax1[0].set_xlim(-1,  10)
    ax1[1].set_xlim(-5,  20)
    ax1[2].set_xlim(-0.01,  0.5)
    ax1[1].xaxis.set_major_locator(plt.MaxNLocator(4))
    plt.suptitle(shape+" 4dof motion")
    colors=[color_purple, color_purple, color_purple, color_purple, color_purple, color_purple, color_purple, color_purple]
    for patch, color in zip(res2['boxes'], colors):
        patch.set_facecolor(color)

    colors=[color_blue, color_blue, color_blue, color_blue, color_blue, color_blue, color_blue, color_blue]
    for patch, color in zip(res1['boxes'], colors):
        patch.set_facecolor(color)

    colors=[color_yellow, color_yellow, color_yellow, color_yellow, color_yellow, color_yellow, color_yellow, color_yellow]
    for patch, color in zip(res3['boxes'], colors):
        patch.set_facecolor(color)

    fig_speeds.subplots_adjust(wspace=0)
    # plt.show()

shape_labels = ['triangle', 'star', 'square', 'puzzle', 'pacman', 'oval', 'flag', 'arrow']
ticks=[0,1,2,3,4,5,6,7]
medianprops = dict(color='k')

vec_x_pos_shapes = []
vec_y_pos_shapes = []
vec_dist_shapes = []
vec_rot_shapes = []
vec_sc_shapes = []

for i in range(len(error_x_all_shapes)):
    vec_x_pos_shapes.append(np.array(error_x_all_shapes[len(error_x_all_shapes)-i-1]))
for i in range(len(error_y_all_shapes)):
    vec_y_pos_shapes.append(np.array(error_y_all_shapes[len(error_y_all_shapes)-i-1]))
for i in range(len(error_dist_all_shapes)):
    vec_dist_shapes.append(np.array(error_dist_all_shapes[len(error_dist_all_shapes)-i-1]))
for i in range(len(error_rot_all_shapes)):
    vec_rot_shapes.append(np.array(error_rot_all_shapes[len(error_rot_all_shapes)-i-1]))
for i in range(len(error_sc_all_shapes)):
    vec_sc_shapes.append(np.array(error_sc_all_shapes[len(error_sc_all_shapes)-i-1]))

# vec_x_pos_shapes = [np.array(error_x_all_shapes[0]), np.array(error_x_all_shapes[1]), np.array(error_x_all_shapes[2]), np.array(error_x_all_shapes[3]), np.array(error_x_all_shapes[4]), np.array(error_x_all_shapes[5]), np.array(error_x_all_shapes[6]), np.array(error_x_all_shapes[7]), np.array(error_x_all_shapes[8]), np.array(error_x_all_shapes[9])]
# vec_y_pos_shapes = [np.array(error_y_all_shapes[0]), np.array(error_y_all_shapes[1]), np.array(error_y_all_shapes[2]), np.array(error_y_all_shapes[3]), np.array(error_y_all_shapes[4]), np.array(error_y_all_shapes[5]), np.array(error_y_all_shapes[6]), np.array(error_y_all_shapes[7]), np.array(error_y_all_shapes[8]), np.array(error_y_all_shapes[9])]
# vec_rot_shapes = [np.array(error_rot_all_shapes[0]), np.array(error_rot_all_shapes[1]), np.array(error_rot_all_shapes[2]), np.array(error_rot_all_shapes[3]), np.array(error_rot_all_shapes[4]), np.array(error_rot_all_shapes[5]), np.array(error_rot_all_shapes[6]), np.array(error_rot_all_shapes[7]), np.array(error_rot_all_shapes[8]), np.array(error_rot_all_shapes[9])]
# vec_sc_shapes = [np.array(error_sc_all_shapes[0]), np.array(error_sc_all_shapes[1]), np.array(error_sc_all_shapes[2]), np.array(error_sc_all_shapes[3]), np.array(error_sc_all_shapes[4]), np.array(error_sc_all_shapes[5]), np.array(error_sc_all_shapes[6]), np.array(error_sc_all_shapes[7]), np.array(error_sc_all_shapes[8]), np.array(error_x_all_shapes[9])]

print(np.mean(error_x_all_shapes[0]))
print(np.mean(error_x_all_shapes[7]))

fig_shapes, ax = plt.subplots(1,3)
ax[0].set_xlabel('Position error [cm]', color='k')
ax[1].set_xlabel('Rotation error [deg]', color='k')
ax[2].set_xlabel('Scale error', color='k')
res1 = ax[0].boxplot(vec_dist_shapes, labels=shape_labels, vert=False, showfliers=False,
                     patch_artist=True, medianprops=medianprops)
res2 = ax[1].boxplot(vec_rot_shapes, vert=False, showfliers=False,
                patch_artist=True, medianprops=medianprops)
res3 = ax[2].boxplot(vec_sc_shapes, vert=False, showfliers=False,
                patch_artist=True, medianprops=medianprops)
for element in ['whiskers', 'fliers', 'means', 'medians']:
    plt.setp(res1[element], color='k')
    plt.setp(res2[element], color='k')
    plt.setp(res3[element], color='k')
# ax[0].set_yticklabels([])
# ax[0].set_yticks([])
ax[1].set_yticklabels([])
ax[1].set_yticks([])
ax[2].set_yticklabels([])
ax[2].set_yticks([])
ax[0].set_xlim(-1,  10)
ax[1].set_xlim(-5,  20)
ax[2].set_xlim(-0.01,  0.5)
ax[1].xaxis.set_major_locator(plt.MaxNLocator(4))

colors=[color_purple, color_purple, color_purple, color_purple, color_purple, color_purple, color_purple, color_purple]
for patch, color in zip(res2['boxes'], colors):
    patch.set_facecolor(color)

colors=[color_blue, color_blue, color_blue, color_blue, color_blue, color_blue, color_blue, color_blue]
for patch, color in zip(res1['boxes'], colors):
    patch.set_facecolor(color)

colors=[color_yellow, color_yellow, color_yellow, color_yellow, color_yellow, color_yellow, color_yellow, color_yellow]
for patch, color in zip(res3['boxes'], colors):
    patch.set_facecolor(color)

# plt.suptitle(shape+" 4dof motion")
# colors=[color_ekom, color_rgbde, color_ekom, color_rgbde, color_ekom, color_rgbde, color_ekom, color_rgbde, color_ekom, color_rgbde, color_ekom, color_rgbde]
# color='white'
# colors = [color, color, color, color, color, color,color, color,color, color,color, color]
# patterns=[0,1,0,1,0,1,0,1,0,1,0,1]
# for patch, color in zip(res1['boxes'], colors):
#     patch.set_facecolor(color)
    # if pattern == 1:
    #     patch.set(hatch = '/')
# for patch, color in zip(res2['boxes'], colors):
#     patch.set_facecolor(color)
    # if pattern == 1:
    #     patch.set(hatch = '/')
# ax1[1].legend([res1["boxes"][0], res1["boxes"][1]], ['EKOM', 'RGB-D-E'], loc='upper center', bbox_to_anchor=(0, -0.2),
#         fancybox=True, shadow=True, ncol=2)
fig_shapes.subplots_adjust(wspace=0)
plt.show()