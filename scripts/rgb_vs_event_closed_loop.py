 
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

# exp 1 rgb same gains as event camera different target speeds

def delete_useless_strings(rgb_exp1_path, file1, file2):
    delete_list = ['"[label: ""x""','size: 15\n', 'stride: 1]"', ',\n']

    with open(os.path.join(rgb_exp1_path, d, file2)) as fin, open(os.path.join(rgb_exp1_path, d, "modified.csv"), "w") as fout:
        for line in fin:
            for word in delete_list:
                line = line.replace(word, "")
            fout.write(line)
        fout.close()

file1 = 'ee_pose.csv'
file2 = 'modified.csv'

files_list = []
directories_list = []

rgb_error_list = []
event_error_list = []

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

    mean_error = np.mean(abs(delta_u))
    event_error_list.append(mean_error)
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
    # plt.show()


    rgb_exp1_data = np.loadtxt(os.path.join(rgb_exp1_path, d, file2), delimiter=",", skiprows=1)[:,:8] 
    timestamp = rgb_exp1_data[:,0]- rgb_exp1_data[0,0]
    delta_u = rgb_exp1_data[:,2]
    delta_v = rgb_exp1_data[:,3]
    delta_scale = rgb_exp1_data[:,4]
    delta_theta = rgb_exp1_data[:,5]

    # if the experiment did not fail cut to have some comparable duration
    if (timestamp[-1]>30):
        index_30 = next(x for x, val in enumerate(timestamp) if val > 30)
        timestamp = timestamp[0:index_30]
        delta_u = delta_u[0:index_30] 
        # print(d + " " + str(mean_error)+" pix")
        index_10 =  next(x for x, val in enumerate(timestamp) if val > 10)
        delta_u = delta_u[0:index_10] 
        mean_error = np.mean(abs(delta_u)) 
        print(d + ", error = ", str(mean_error))

    else:
        print(d + ", failed, stopped at t = "+ str(timestamp[-1])+" s")

        plt.figure()
        plt.plot(timestamp, delta_u+320, label='u')
        plt.plot(timestamp, delta_v+240, label='v')
        plt.plot(timestamp, delta_theta, label='theta')
        plt.plot(timestamp, delta_scale+1, label='scale')
        plt.xlabel('Time [s]')
        plt.ylabel('Delta theta [px]')
        plt.title('Target speed ='+d)
        plt.legend()
        plt.show()

    # v_x_filtered, v_y_filtered, v_z_filtered, v_yaw_filtered,  Kv_y, Kv_x, Kv_z, Kv_yaw, filter, filter_z, filter_ang

    rgb_error_list.append(mean_error)

speeds = ['V1', 'V2', 'V3', 'V4', 'V5', 'V6']

