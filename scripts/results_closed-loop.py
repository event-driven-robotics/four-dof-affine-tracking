 
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

exp1_path = "combined_results/exp1"
exp1_files = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), exp1_path)) if file.endswith('.txt')])
exp1_data_list = []

for f in tqdm(exp1_files, "Loading exp1 data"):
    exp1_data = np.loadtxt(os.path.join(exp1_path, f), delimiter=" ", skiprows=2)[:,:11] # delete the first two lines
    exp1_data_list.append(exp1_data)

mean_rot_errors = []
mean_scale_errors = []
mean_trans_x_errors = []
mean_trans_y_errors = []

row = 0 
column = 0

fig_error_time, ax_error_time = plt.subplots(4, 6)

for idx, x in enumerate(exp1_data_list):
    
    if (idx>=0 and idx<6):
        mean_rot_errors.append(np.mean(abs(exp1_data_list[idx][:, 5])))
        ax_error_time[0, idx].plot(exp1_data_list[idx][:, 0], exp1_data_list[idx][:, 5])

    if (idx>=6 and idx<12):
        mean_scale_errors.append(np.mean(abs(exp1_data_list[idx][:, 6]-1)))
        ax_error_time[1, idx-6].plot(exp1_data_list[idx][:, 0], exp1_data_list[idx][:, 6]-1)

    if (idx>=12 and idx<18):
        mean_trans_x_errors.append(np.mean(abs(exp1_data_list[idx][:, 3]-320)))
        ax_error_time[2, idx-18].plot(exp1_data_list[idx][:, 0], exp1_data_list[idx][:, 3]-320)

    if (idx>=18 and idx<24):
        mean_trans_y_errors.append(np.mean(abs(exp1_data_list[idx][:, 4]-240)))
        ax_error_time[3, idx-24].plot(exp1_data_list[idx][:, 0], exp1_data_list[idx][:, 4]-240)

plt.setp(ax_error_time[-1, :], xlabel='Time [s]')
plt.setp(ax_error_time[0, 0], ylabel='theta \n [deg]')
plt.setp(ax_error_time[1, 0], ylabel='s')
plt.setp(ax_error_time[2,0], ylabel='x \n [pix]')
plt.setp(ax_error_time[3,0], ylabel='y \n [pix]')

rot_speeds=["100","150", "175", "200", "225", "250"]
scale_speeds=["30", "40", "50", "60", "70", "80"]
trans_x_speeds=["240", "480", "720", "960", "1200", "1440"]
trans_y_speeds=["135", "270", "405", "540", "675", "810"]

n_trans_x_speeds = np.arange(len(trans_x_speeds))
n_trans_y_speeds = np.arange(len(trans_y_speeds))
n_rot_speeds = np.arange(len(rot_speeds))
n_scale_speeds = np.arange(len(scale_speeds))

fig, ax = plt.subplots()
ax.bar(n_rot_speeds, mean_rot_errors, align='center', ecolor='black', alpha=0.5, capsize=10, color='tab:green')
ax.set_ylabel('Rotation Error \n [deg]')
ax.set_xlabel('Target Speed [Hz]')
ax.set_xticks(n_rot_speeds)
ax.set_xticklabels(rot_speeds)
ax.yaxis.grid(True)

fig, ax = plt.subplots()
ax.bar(n_scale_speeds, mean_scale_errors, align='center', ecolor='black', alpha=0.5, capsize=10, color='tab:red')
ax.set_ylabel('Scale Error')
ax.set_xlabel('Target Speed \n [Hz]')
ax.set_xticks(n_scale_speeds)
ax.set_xticklabels(scale_speeds)
ax.yaxis.grid(True)

fig, ax = plt.subplots()
ax.bar(n_trans_x_speeds, mean_trans_x_errors, align='center', ecolor='black', alpha=0.5, capsize=10, color='tab:blue')
ax.set_ylabel('Translation Error x \n [pix]')
ax.set_xlabel('Target Speed [Hz]')
ax.set_xticks(n_trans_x_speeds)
ax.set_xticklabels(trans_x_speeds)
ax.yaxis.grid(True)

fig, ax = plt.subplots()
ax.bar(n_trans_y_speeds, mean_trans_y_errors, align='center', ecolor='black', alpha=0.5, capsize=10, color='tab:orange')
ax.set_ylabel('Translation Error y \n [pix]')
ax.set_xlabel('Target Speed [Hz]')
ax.set_xticks(n_trans_y_speeds)
ax.set_xticklabels(trans_y_speeds)
ax.yaxis.grid(True)

# plt.show()

exp2_path = "combined_results/exp2"
exp2_files = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), exp2_path)) if file.endswith('.txt')])
exp2_data_list = []

for f in tqdm(exp2_files, "Loading exp2 data"):
    exp2_data = np.loadtxt(os.path.join(exp2_path, f), delimiter=" ", skiprows=2)[:,:11] # delete the first two lines
    exp2_data_list.append(exp2_data)

latency_0_mean_error = []
latency_5_mean_error = []
latency_10_mean_error = []
latency_15_mean_error = []
latency_20_mean_error = []
latency_25_mean_error = []
latency_30_mean_error = []
latency_35_mean_error = []
latency_40_mean_error = []
latency_45_mean_error = []
latency_50_mean_error = []

fig_error_time_latency_0, ax_error_time_latency_0 = plt.subplots(4,1)
fig_error_time_latency_5, ax_error_time_latency_5 = plt.subplots(4,1)
fig_error_time_latency_10, ax_error_time_latency_10 = plt.subplots(4,1)
fig_error_time_latency_15, ax_error_time_latency_15 = plt.subplots(4,1)
fig_error_time_latency_20, ax_error_time_latency_20 = plt.subplots(4,1)
fig_error_time_latency_25, ax_error_time_latency_25 = plt.subplots(4,1)
fig_error_time_latency_30, ax_error_time_latency_30 = plt.subplots(4,1)
fig_error_time_latency_35, ax_error_time_latency_35 = plt.subplots(4,1)
fig_error_time_latency_40, ax_error_time_latency_40 = plt.subplots(4,1)
fig_error_time_latency_45, ax_error_time_latency_45 = plt.subplots(4,1)
fig_error_time_latency_50, ax_error_time_latency_50 = plt.subplots(4,1)

for idx, x in enumerate(exp2_data_list):
    
    if (idx>=0 and idx<4):
        latency_0_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_0[idx].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="speed #"+str(idx+1))
        plt.setp(ax_error_time_latency_0[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_0[:], ylabel='Error [pix]')
        ax_error_time_latency_0[idx].legend(loc="upper right")
        ax_error_time_latency_0[idx].set_title("Latency = 0 ms")   

    if (idx>=4 and idx<8):
        latency_5_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_5[idx-4].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="speed #"+str(idx-4+1))
        plt.setp(ax_error_time_latency_5[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_5[:], ylabel='Error [pix]')
        ax_error_time_latency_5[idx-4].legend(loc="upper right")
        ax_error_time_latency_5[idx-4].set_title("Latency = 5 ms")

    if (idx>=8 and idx<12):
        latency_10_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_10[idx-8].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="speed #"+str(idx-8+1))
        plt.setp(ax_error_time_latency_10[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_10[:], ylabel='Error [pix]')
        ax_error_time_latency_10[idx-8].legend(loc="upper right")
        ax_error_time_latency_10[idx-8].set_title("Latency = 10 ms")

    if (idx>=12 and idx<16):
        latency_15_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_15[idx-12].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="speed #"+str(idx-12+1))
        plt.setp(ax_error_time_latency_15[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_15[:], ylabel='Error [pix]')
        ax_error_time_latency_15[idx-12].legend(loc="upper right")
        ax_error_time_latency_15[idx-12].set_title("Latency = 15 ms")

    if (idx>=16 and idx<20):
        latency_20_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_20[idx-16].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="speed #"+str(idx-16+1))
        plt.setp(ax_error_time_latency_20[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_20[:], ylabel='Error [pix]')
        ax_error_time_latency_20[idx-16].legend(loc="upper right")
        ax_error_time_latency_20[idx-16].set_title("Latency = 20 ms")

    if (idx>=20 and idx<24):
        latency_25_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_25[idx-20].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="speed #"+str(idx+1-20))
        plt.setp(ax_error_time_latency_25[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_25[:], ylabel='Error [pix]')
        ax_error_time_latency_25[idx-20].legend(loc="upper right")
        ax_error_time_latency_25[idx-20].set_title("Latency = 25 ms")

    if (idx>=24 and idx<28):
        latency_30_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_30[idx-24].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="speed #"+str(idx+1-24))
        plt.setp(ax_error_time_latency_30[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_30[:], ylabel='Error [pix]')
        ax_error_time_latency_30[idx-24].legend(loc="upper right")
        ax_error_time_latency_30[idx-24].set_title("Latency = 30 ms")

    if (idx>=28 and idx<32):
        latency_35_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_35[idx-28].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="35 ms")
        ax_error_time_latency_35[idx-28].plot(exp2_data_list[idx-28][:, 0], exp2_data_list[idx-28][:, 3]-320, label="0 ms")
        f = interpolate.interp1d(exp2_data_list[idx-28][:, 0], exp2_data_list[idx-28][:, 3]-320, kind='nearest',fill_value="extrapolate")
        y_0_28 = f(exp2_data_list[idx][:, 0])
        diff_0_35_lat = abs(exp2_data_list[idx][:, 3]-320 - y_0_28)
        mean_diff_0_35_lat = np.mean(diff_0_35_lat)
        # max_diff = 
        index_time_failure_35 = next(x for x, val in enumerate(diff_0_35_lat) if val > (mean_diff_0_35_lat))
        ax_error_time_latency_35[idx-28].axhline(y = mean_diff_0_35_lat, color = 'k')
        ax_error_time_latency_35[idx-28].axvline(x = exp2_data_list[idx][index_time_failure_35, 0], color = 'r')
        ax_error_time_latency_35[idx-28].plot(exp2_data_list[idx][:, 0], diff_0_35_lat, label="error")
        plt.setp(ax_error_time_latency_35[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_35[:], ylabel='Error [pix]')
        ax_error_time_latency_35[idx-28].legend(loc="upper right")
        ax_error_time_latency_45[idx-28].set_title("speed #"+str(idx+1-28))

    if (idx>=32 and idx<36):
        latency_40_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_40[idx-32].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="40 ms")
        ax_error_time_latency_40[idx-32].plot(exp2_data_list[idx-32][:, 0], exp2_data_list[idx-32][:, 3]-320, linestyle='dashed', label="0 ms")
        plt.setp(ax_error_time_latency_40[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_40[:], ylabel='Error [pix]')
        ax_error_time_latency_40[idx-32].legend(loc="upper right")
        ax_error_time_latency_40[idx-32].set_title("speed #"+str(idx+1-32))

    if (idx>=36 and idx<40):
        latency_45_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_45[idx-36].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="45 ms")
        ax_error_time_latency_45[idx-36].plot(exp2_data_list[idx-36][:, 0], exp2_data_list[idx-36][:, 3]-320, linestyle='dashed', label = "0 ms")
        plt.setp(ax_error_time_latency_45[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_45[:], ylabel='Error [pix]')
        ax_error_time_latency_45[idx-36].legend(loc="upper right")
        ax_error_time_latency_45[idx-36].set_title("speed #"+str(idx+1-36))

    if (idx>=40 and idx<44):
        latency_50_mean_error.append(np.mean(abs(exp2_data_list[idx][:, 3]-320)))
        ax_error_time_latency_50[idx-40].plot(exp2_data_list[idx][:, 0], exp2_data_list[idx][:, 3]-320, label="50 ms")
        ax_error_time_latency_50[idx-40].plot(exp2_data_list[idx-40][:, 0], exp2_data_list[idx-40][:, 3]-320, linestyle='dashed', label="0 ms")
        plt.setp(ax_error_time_latency_50[-1], xlabel='Time [s]')
        plt.setp(ax_error_time_latency_50[:], ylabel='Error [pix]')
        ax_error_time_latency_50[idx-40].legend(loc="upper right")
        ax_error_time_latency_50[idx-40].set_title("speed #"+str(idx+1-40))

plt.legend()

latency_array = [0,5,10,15,20,25,30,35,40,45,50]
speed_array = [240,480,720,960]

speed = 0
speed_240 = [latency_0_mean_error[speed], latency_5_mean_error[speed], latency_10_mean_error[speed], latency_15_mean_error[speed], latency_20_mean_error[speed], latency_25_mean_error[speed], latency_30_mean_error[speed], latency_35_mean_error[speed], latency_40_mean_error[speed], latency_45_mean_error[speed], latency_50_mean_error[speed]]
speed = 1
speed_480 = [latency_0_mean_error[speed], latency_5_mean_error[speed], latency_10_mean_error[speed], latency_15_mean_error[speed], latency_20_mean_error[speed], latency_25_mean_error[speed], latency_30_mean_error[speed], latency_35_mean_error[speed], latency_40_mean_error[speed], latency_45_mean_error[speed], latency_50_mean_error[speed]]
speed = 2
speed_720 = [latency_0_mean_error[speed], latency_5_mean_error[speed], latency_10_mean_error[speed], latency_15_mean_error[speed], latency_20_mean_error[speed], latency_25_mean_error[speed], latency_30_mean_error[speed], latency_35_mean_error[speed], latency_40_mean_error[speed], latency_45_mean_error[speed], latency_50_mean_error[speed]]
speed = 3
speed_960 = [latency_0_mean_error[speed], latency_5_mean_error[speed], latency_10_mean_error[speed], latency_15_mean_error[speed], latency_20_mean_error[speed], latency_25_mean_error[speed], latency_30_mean_error[speed], latency_35_mean_error[speed], latency_40_mean_error[speed], latency_45_mean_error[speed], latency_50_mean_error[speed]]


fig_lat = plt.figure()

plt.plot(latency_array, speed_240, label='240 Hz')
plt.plot(latency_array, speed_480, label='480 Hz')
plt.plot(latency_array, speed_720, label='720 Hz')
plt.plot(latency_array, speed_960, label='960 Hz')

plt.title("Translation x")
plt.xlabel("Latency [ms]")
plt.ylabel("Error [pix]")

plt.legend()

plt.show() 