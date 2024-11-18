 
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
          'figure.figsize': [15, 4.8],  # ratio 1.6 is pleasant
          'axes.titlesize': 13,
          'axes.labelsize': 28,
          'lines.linewidth': 2,
          'lines.markersize': 4,
          'legend.fontsize': 20}

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)

data_recording_time = 10


import math

def radians_to_degrees(radians_list):
    """
    Convert a list of angles from radians to degrees.

    Args:
    - radians_list: A list of angles in radians.

    Returns:
    - A list of angles in degrees.
    """
    return [math.degrees(angle) for angle in radians_list]

def adjust_yaw_trajectory(yaw_angles):
    """
    Adjusts a yaw trajectory where the angle passes instantaneously from 0 to 180 degrees.
    After detecting this jump, it subtracts all subsequent values from the last value before the jump.

    Args:
    - yaw_angles: A list of yaw angles (in degrees).

    Returns:
    - A modified list of yaw angles after applying the correction.
    """
    adjusted_yaw_angles = []
    reached = False
    
    for i in range(len(yaw_angles)):
        current_yaw = yaw_angles[i]

        
        # Detect instantaneous jump from 0 to 180 (with a threshold around 0 and a sudden jump to 180)
        if i > 0 and current_yaw < 60:
            reached = True
        else:
            reached = False
            
                
        if reached is False:
            # No jump detected yet, append the current yaw angle to the adjusted list
            adjusted_yaw_angles.append(current_yaw)
        else:
            # After detecting the jump, subtract the current value from last_value_before_jump
            adjusted_yaw_angles.append(-current_yaw + 180)
    
    return adjusted_yaw_angles
############################### event vs rgb same gain different target speeds ########################################################

file1 = 'ee_pose.csv'

files_list = []
directories_list = []

event_error_list = []
event_std_error_list = []

real_obj_path = "/home/luna/Downloads/bags_mustard3/bags/mustard-28-10-3_2024-10-28-16-07-02"
# real_obj_path = "/home/luna/Downloads/bag_table/mustard-10-10-4_2024-10-10-17-13-51"
# real_obj_path = "/home/luna/Downloads/bag_random/mustard-10-10-2_2024-10-10-16-49-56"

real_obj_ee_pose = np.loadtxt(os.path.join(real_obj_path, file1), delimiter=",", skiprows=1)
time_real_obj_ee_pose = real_obj_ee_pose[:,0]-real_obj_ee_pose[0,0]
x_real_obj_ee_pose = real_obj_ee_pose[:,1]
y_real_obj_ee_pose = real_obj_ee_pose[:,2]
z_real_obj_ee_pose = real_obj_ee_pose[:,3]
rx_real_obj_ee_pose = real_obj_ee_pose[:,4]
ry_real_obj_ee_pose = real_obj_ee_pose[:,5]
rz_real_obj_ee_pose = real_obj_ee_pose[:,6]
rw_real_obj_ee_pose = real_obj_ee_pose[:,7]
index_start =0 
index_start = next(x for x, val in enumerate(y_real_obj_ee_pose) if val > -0.32)
time_real_obj_ee_pose = time_real_obj_ee_pose[index_start:-1]
time_real_obj_ee_pose = time_real_obj_ee_pose-time_real_obj_ee_pose[0]
x_real_obj_ee_pose = x_real_obj_ee_pose[index_start:-1]
y_real_obj_ee_pose = y_real_obj_ee_pose[index_start:-1]
z_real_obj_ee_pose = z_real_obj_ee_pose[index_start:-1]
rx_real_obj_ee_pose = rx_real_obj_ee_pose[index_start:-1]
ry_real_obj_ee_pose = ry_real_obj_ee_pose[index_start:-1]
rz_real_obj_ee_pose = rz_real_obj_ee_pose[index_start:-1]
rw_real_obj_ee_pose = rw_real_obj_ee_pose[index_start:-1]
index_end = -1
index_end = next(x for x, val in enumerate(time_real_obj_ee_pose) if val > 14)
rx_real_obj_ee_pose = rx_real_obj_ee_pose[0:index_end]
ry_real_obj_ee_pose = ry_real_obj_ee_pose[0:index_end]
rz_real_obj_ee_pose = rz_real_obj_ee_pose[0:index_end]
rw_real_obj_ee_pose = rw_real_obj_ee_pose[0:index_end]

rx_real_obj_ee_pose_deg = radians_to_degrees(rx_real_obj_ee_pose)
cleaned_yaw_angles = adjust_yaw_trajectory(rx_real_obj_ee_pose_deg)

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()
ax1.plot(time_real_obj_ee_pose[0:index_end], x_real_obj_ee_pose[0:index_end], label = 'x', color='red')
ax1.plot(time_real_obj_ee_pose[0:index_end], y_real_obj_ee_pose[0:index_end], label = 'y', color='green')
ax1.plot(time_real_obj_ee_pose[0:index_end], z_real_obj_ee_pose[0:index_end], label = 'z', color='blue')
ax2.plot(time_real_obj_ee_pose[0:index_end], cleaned_yaw_angles, label = 'yaw', color='black', alpha = 0.2)
# ax2.plot(time_real_obj_ee_pose[0:index_end], rx_real_obj_ee_pose_deg, label = 'yaw', color='black')

ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Position [m]')
ax2.set_ylim([80, 250])
# ax2.set_ylabel('Rotation \n [deg]')
ax1.legend(loc='lower left', bbox_to_anchor= (1.1, 0.5), ncol=1,
            borderaxespad=2, frameon=False)
ax2.legend(loc='lower left', bbox_to_anchor= (1.1, 0.38), ncol=1,
            borderaxespad=2, frameon=False)
plt.show()