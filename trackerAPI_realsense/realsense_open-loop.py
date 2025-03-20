import os
import numpy as np
from tqdm import tqdm
from scipy import interpolate
import matplotlib 
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator
import cv2

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

def compute_error(t_list, gt_list, t_track_list, x_track_list):

    error_list = []

    f = interpolate.interp1d(t_track_list, x_track_list, kind='nearest',fill_value="extrapolate")

    y_ = f(t_list)

            
    diff = abs(y_ - gt_list)

    meanerror = np.mean(diff)

    return meanerror

def computeEuclideanDistance(errors_x_list, errors_y_list):
    distances = []
    for i in range(len(errors_x_list)):
        distances.append(np.sqrt(np.multiply(np.array(errors_x_list[i]),np.array(errors_x_list[i]))+np.multiply(np.array(errors_y_list[i]),np.array(errors_y_list[i]))))

    return distances

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

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

posList = []
def onMouse(event, x, y, flags, param):
   global posList
   if event == cv2.EVENT_LBUTTONDOWN:
        print((x,y))
        posList.append((x, y))

ground_truth_trans_x = np.hstack((np.arange(1, 768+1, 1), np.arange(767, -769, -1), np.arange(-767, 1, 1)))
speeds_trans_x = [240,480,720,960,1200,1440]
gt_trans_x_list, t_trans_x_list = generate_ground_truth(speeds_trans_x, ground_truth_trans_x)
ground_truth_trans_x_y_monitor = []
for idx, value in enumerate(gt_trans_x_list):
    ground_truth_trans_x_y_monitor.append((1080/2)*np.ones(len(value)))

# for x in range(6):
#     plt.figure()
#     plt.plot(t_trans_x_list[x], gt_trans_x_list[x])

# plt.show()

rgb_path = "/home/luna/code/workbook_marco-monforte/trackerAPI_realsense/build/real_sense_trans_x-open-loop/"

# rgb_images = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), rgb_path)) if file.endswith('.jpg')])

# np.set_printoptions(suppress=True)

# count = 0
# for f in tqdm(rgb_images, "Loading images and print homography to file:"):
#     posList = []
#     img = cv2.imread(os.path.join(rgb_path, f))
#     cv2.imshow("image", img)
#     cv2.setMouseCallback('image', onMouse)
#     cv2.waitKey(0)
#     pts_dst = np.array(posList)     # convert to NumPy for later use

#     pts_src = np.array([[0,0],[1920,0], [1920,1080], [0,1080]])

#     h, status = cv2.findHomography(pts_dst, pts_src)

#     with open(os.path.join(rgb_path, f + "_homography.csv"), "w") as f:
#         for line in h:
#             modified_line = str(line).replace('[', '').replace(']', '')
#             f.write(modified_line)
#             f.write('\n')
#         f.close()

rgb_homographies = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), rgb_path)) if file.endswith('.csv')])
H_list = []
for f in tqdm(rgb_homographies, "Loading homographies and store to a list:"):
    lines = []
    with open(os.path.join(rgb_path, f)) as linefile:
        for row in range(3):
            line = linefile.readline().strip('\n')
            line_list = list(map(float, line.split(' ')))
            lines.append(line_list)
            
        H_list.append(np.array(lines))

rgb_tracking = sorted([file for file in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), rgb_path)) if file.endswith('.txt')])
idx = 0
for file in tqdm(rgb_tracking, "Loading rgb tracking results:"):
    rgb_data = np.loadtxt(os.path.join(rgb_path, file), delimiter=" ", skiprows=3)[:,:3] 

    u_gt = []
    v_gt = []
    for i in range(len(gt_trans_x_list[idx])):
        
        coord_gt_monitor_plane = np.array([gt_trans_x_list[idx][i]+1920/2, ground_truth_trans_x_y_monitor[idx][i],1])

        result_gt = np.matmul(np.linalg.inv(H_list[idx]), coord_gt_monitor_plane)
        u_gt.append(result_gt[0]/result_gt[2])
        v_gt.append(result_gt[1]/result_gt[2])

    fig = plt.figure()
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    plt.plot(rgb_data[:,0], rgb_data[:,1])
    plt.plot(t_trans_x_list[idx], u_gt)
    plt.show()

    cleaning_start_time = coords[0][0]
    index_start = find_nearest(rgb_data[:,0], cleaning_start_time)

    rgb_data = rgb_data[index_start:-1,:]
    rgb_data[:,0] = rgb_data[:,0]-rgb_data[0,0]

    index_end = next(x for x, val in enumerate(rgb_data[:,0]) if val > 10)
    rgb_time_10s = rgb_data[:index_end,0] 
    x_rgb_data=rgb_data[:index_end,1]    
    y_rgb_data=rgb_data[:index_end,2]

    f = interpolate.interp1d(rgb_time_10s, x_rgb_data, kind='nearest',fill_value="extrapolate")
    f2 = interpolate.interp1d(rgb_time_10s, y_rgb_data, kind='nearest',fill_value="extrapolate")

    y_ = f(t_trans_x_list[idx])
    y_2 = f2(t_trans_x_list[idx])

        
    diff = abs(y_ - u_gt)

    meanerror = np.mean(diff)
    print(meanerror)

    coords = []


    fig = plt.figure()
    plt.plot(t_trans_x_list[idx], y_)
    plt.plot(t_trans_x_list[idx], u_gt)
    plt.show()

    fig = plt.figure()
    plt.plot(t_trans_x_list[idx], y_2)
    plt.plot(t_trans_x_list[idx], v_gt)
    plt.show()

    error = compute_error(t_trans_x_list[idx],u_gt, rgb_time_10s, x_rgb_data )

    print(file)
    print(str(error))

    idx = idx+1