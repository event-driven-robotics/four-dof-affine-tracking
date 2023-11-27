 
import os
import numpy as np
from tqdm import tqdm
import matplotlib 
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator
from scipy import interpolate
import csv
from itertools import zip_longest
import math 

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

def quaternion_to_euler_angle(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """

        roll_list = []
        pitch_list = []
        yaw_list = []

        degrees = 57.2958

        for qx,qy,qz,qw in zip(x,y,z,w):

            t0 = +2.0 * (qw * qx + qy * qz)
            t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (qw * qy - qz * qx)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (qw * qz + qx * qy)
            t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
            yaw_z = math.atan2(t3, t4)

            roll_list.append(roll_x*degrees)
            pitch_list.append(pitch_y*degrees)  
            yaw_list.append(yaw_z*degrees)

            roll_array = np.array(roll_list)
            pitch_array = np.array(pitch_list)
            yaw_array = np.array(yaw_list)
     
        return roll_array, pitch_array, yaw_array # in degrees

def cleanEuler(angle, angle_type): 

    # angle = angle[~np.isnan(angle)]
    diff_arrays = angle[1:-1]-angle[0:-2]
    prev_x = 0
    th = 180
    filtered_angles_list = [] 
    diff_list=[]
    for idx, x in enumerate(angle):
        if idx == 0:
            if angle_type==2 and x < 0:
                x+=360 
            prev_x = x
        else:
            diff = abs(x - prev_x)
            diff_list.append(diff)
            if diff > th:
                x += 360
            else:
                if angle_type==2 and x<0:
                    x += 360
            prev_x = x
        filtered_angles_list.append(x)

    return(np.array(filtered_angles_list))

def load_ground_truth(filename):
    groud_truth_combined_motions = np.loadtxt(filename, delimiter=" ")
    groud_truth_combined_motions[:,0] = groud_truth_combined_motions[:,0]+1920/2
    groud_truth_combined_motions[:,1] = groud_truth_combined_motions[:,1]+1080/2
    return groud_truth_combined_motions

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

ground_truth_combined = load_ground_truth("/usr/local/src/affine2dtracking/gt_comb/gt_combined_motions.txt")
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

print("gt generated")

gt_800 = np.loadtxt("/usr/local/src/affine2dtracking/gt_comb_800.csv", delimiter=",", skiprows=1)[:,:6] 

############################### event vs rgb same gain different target speeds ########################################################

file1 = 'ee_pose.csv'
file2 = 'modified.csv'
file3 = 'errors_and_velocities.csv'

event_4dof_path = "/usr/local/src/affine2dtracking/bag_files/event-camera-different-motions/combined_800/"

event_exp1_data = np.loadtxt(os.path.join(event_4dof_path, file2), delimiter=",", skiprows=1)[:,:8] 
timestamp = event_exp1_data[:,0]- event_exp1_data[0,0]
delta_u = event_exp1_data[:,2]

event_exp1_ee_pose = np.loadtxt(os.path.join(event_4dof_path, file1), delimiter=",", skiprows=1)[:,:8] 
time_ee_pose = event_exp1_ee_pose[:,0] - event_exp1_ee_pose[0,0]

print("data loaded")

fig, ax = plt.subplots(figsize=(15, 8))
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.plot(time_ee_pose, event_exp1_ee_pose[:,1], label='x')
plt.plot(time_ee_pose, event_exp1_ee_pose[:,2], label='y')
plt.plot(time_ee_pose, event_exp1_ee_pose[:,3], label='z')
# plt.plot(time_ee_pose, gamma_cleaned, label='yaw')
plt.legend()
plt.show()

cleaning_start_time = coords[0][0]
cleaning_end_time = coords[1][0]

index_start = find_nearest(time_ee_pose, cleaning_start_time)
index_end = find_nearest(time_ee_pose, cleaning_end_time)

index_start_du = find_nearest(timestamp, cleaning_start_time)
index_end_du = find_nearest(timestamp, cleaning_end_time)

time_ee_pose = time_ee_pose[index_start:index_end]
time_ee_pose = time_ee_pose - time_ee_pose[0]

timestamp = timestamp[index_start:index_end]
timestamp = timestamp - timestamp[0]

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

alpha, beta, gamma = quaternion_to_euler_angle(event_exp1_ee_pose[index_start:index_end,7], event_exp1_ee_pose[index_start:index_end,4], event_exp1_ee_pose[index_start:index_end,5], event_exp1_ee_pose[index_start:index_end,6])

alpha2, beta2, gamma2 = quaternion_to_euler_angle(event_exp1_ee_pose[index_start:index_end,4], event_exp1_ee_pose[index_start:index_end,5], event_exp1_ee_pose[index_start:index_end,6], event_exp1_ee_pose[index_start:index_end,7])


print("to euler")

alpha_cleaned = cleanEuler(alpha,0)
beta_cleaned = cleanEuler(beta,1)
gamma_cleaned = cleanEuler(gamma,2)

print("euler cleaned")

fig, ax1 = plt.subplots()
ax1.plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,1], label='x')
ax1.plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,2], label='y')
ax1.plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,3], label='z')
ax1.plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,4], label='qx')
ax1.plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,5], label='qy')
ax1.plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,6], label='qz')
ax1.plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,7], label='qw')
# ax2.plot(time_ee_pose, gamma_cleaned, label='yaw')
plt.legend()
plt.show()


fig, ax1 = plt.subplots(4, 1, figsize=(18, 8))
ax2 = ax1[0].twinx()
ax3 = ax1[1].twinx()
ax4 = ax1[2].twinx()
ax5 = ax1[3].twinx()
# print(high_rgb_ee_pose[0])

# high_rgb_ee_pose = high_rgb_ee_pose + high_event_ee_pose[0]

ax1[0].plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,2], label = r'$y_{ee}$', color='tab:red')
ax2.plot(t_comb_speeds[5][0], -(gt_comb_speeds[5][0]-1920/2), label = r'$u_{gt}$',ls='-.', color = 'tab:red')
ax1[1].plot(time_ee_pose, event_exp1_ee_pose[index_start:index_end,1], label = r'$x_{ee}$', color='tab:green')
ax3.plot(t_comb_speeds[5][1], -(gt_comb_speeds[5][1]-1080/2), label = r'$v_{gt}$',ls='-.', color = 'tab:green')
ax1[2].plot(time_ee_pose[0:-1000], event_exp1_ee_pose[index_start:(index_end-1000),3], label = r'$z_{ee}$', color='tab:blue')
ax4.plot(gt_800[:,0], gt_800[:,4], label = r'$s_{gt}$',ls='-.', color = 'tab:blue')
ax1[3].plot(time_ee_pose, beta2-beta2[0], label = r'$\theta_{ee}$', color='tab:purple')
ax5.plot(t_comb_speeds[5][3], gt_comb_speeds[5][3], label = r'$\theta_{gt}$',ls='-.', color = 'tab:purple')
# ax1[1].plot(medium_rgb_ee_time, medium_rgb_ee_pose, label = 'real-sense',  ls='--', color='tab:orange')
# ax1[1].plot(medium_event_ee_time, medium_event_ee_pose-medium_event_ee_pose[0], label = 'event camera', color='tab:blue')
# ax3.plot(t_trans_x_list_pix[3], -gt_trans_x_list_pix[3], label = 'ground truth', color = 'tab:green')
# ax1[2].plot(high_rgb_ee_time, high_rgb_ee_pose, label = 'real-sense', ls='--', color='tab:orange')
# ax1[2].plot(high_event_ee_time, high_event_ee_pose-high_event_ee_pose[0], label = 'event camera', color='tab:blue')
# ax4.plot(t_trans_x_list_pix[5], -gt_trans_x_list_pix[5], label = 'ground truth', color = 'tab:green')
ax1[0].set_ylabel('[m]')
ax1[1].set_ylabel('[m]')
ax1[2].set_ylabel('[m]')
ax1[3].set_ylabel('[deg]')
ax2.set_ylabel('[px]')
ax3.set_ylabel('[px]')
ax5.set_ylabel('[deg]')
ax1[3].set_xlabel('Time [s]')
# ax1[0].set_ylabel('Position [m]')
# ax2.set_ylabel('Position [px]')
# ax1[1].set_ylabel('\nEnd-effector position')
# ax3.set_ylabel('\nTarget state')
# ax1[2].set_ylabel('Position [m]')
# ax4.set_ylabel('Position [px]')
# ax1[0].set_xlim([0, 10])
# ax1[1].set_xlim([0, 10])
# ax1[2].set_xlim([0, 10])
lines, labels = ax1[0].get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, loc=0)
lines, labels = ax1[1].get_legend_handles_labels()
lines3, labels3 = ax3.get_legend_handles_labels()
ax3.legend(lines + lines3, labels + labels3, loc=0)
lines, labels = ax1[2].get_legend_handles_labels()
lines4, labels4 = ax4.get_legend_handles_labels()
ax4.legend(lines + lines4, labels + labels4, loc="upper right")
lines, labels = ax1[3].get_legend_handles_labels()
lines5, labels5 = ax5.get_legend_handles_labels()
ax5.legend(lines + lines5, labels + labels5, loc=0)
# ax[0].set_legend(loc = 'upper right')
plt.show()


