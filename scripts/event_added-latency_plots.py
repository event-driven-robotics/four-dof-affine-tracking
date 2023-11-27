 
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
          'legend.fontsize': 20}

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)

latencies_1440 = ['0', '5', '10', '15', '20', '25', '30', '35', '50', '55', '60', '65', '80']

latencies_240 = ['0', '5', '10', '15', '20', '25', '30', '35','40','45','50']

mean_error_1440 = [59.96098149637972, 59.906878519710375, 62.15633802816902, 60.40611178126257, 61.270699356913184,  63.35974261009451, 65.10467571644043, 67.49017236165709, 72.41933862699769, 75.39682220434433, 80, 80, 80]
time_to_failure_1440 = [10,10,10,10,10,10,10,10,10,10, 8.387627173573163,6.288995189622945,1.0765299476920749]

# 0240hz_00ms.txt NO FAILURE, mean=7.075706214689266, std=5.1669776852204325
# 0240hz_05ms.txt NO FAILURE, mean=5.253868345135064, std=2.827233314479978
# 0240hz_10ms.txt NO FAILURE, mean=7.095238095238095, std=5.039499200830898
# 0240hz_15ms.txt NO FAILURE, mean=7.673804634790261, std=5.5891733846831135
# 0240hz_20ms.txt NO FAILURE, mean=8.70402703463813, std=6.334653287779384
# 0240hz_25ms.txt NO FAILURE, mean=8.541754597527886, std=4.979774195337758
# 0240hz_30ms.txt NO FAILURE, mean=9.091073582629674, std=6.32456714019784
# 0240hz_35ms.txt NO FAILURE, mean=8.6070323488045, std=5.251803633421269
# 0240hz_40ms.txt NO FAILURE, mean=10.134053738317757, std=5.968410546791844
# 0240hz_45ms.txt FAILED AT 4.91250698091757 162.3916861826698
# 0240hz_50ms.txt FAILED AT 5.011710173760541 56.55031731640979
# 0240hz_45ms 5.873236869365439
# 0240hz_50ms 5.854693051520789
# 0240hz_45ms 5.941436700765214
# 0240hz_50ms 5.854693051520789

time_to_failure_240 = [10,10,10,10,10,10,10,10,10,5.801675941823827, 5.227823840593588]
mean_error_240 = [7.075706214689266, 5.253868345135064, 7.095238095238095, 7.673804634790261, 8.70402703463813, 8.541754597527886, 9.091073582629674, 8.6070323488045, 10.134053738317757, 15, 15]
std_error_240 = [5.1669776852204325, 2.827233314479978, 5.039499200830898, 5.5891733846831135, 6.334653287779384, 4.979774195337758, 6.32456714019784, 5.251803633421269, 5.968410546791844, 0, 0]

diff_std_240 = [mean_error_240[0]-std_error_240[0], mean_error_240[1]-std_error_240[1], mean_error_240[2]-std_error_240[2], mean_error_240[3]-std_error_240[3], mean_error_240[4]-std_error_240[4], mean_error_240[5]-std_error_240[5], mean_error_240[6]-std_error_240[6], mean_error_240[7]-std_error_240[7], mean_error_240[8]-std_error_240[8]]
sum_std_240 = [mean_error_240[0]+std_error_240[0], mean_error_240[1]+std_error_240[1], mean_error_240[2]+std_error_240[2], mean_error_240[3]+std_error_240[3], mean_error_240[4]+std_error_240[4], mean_error_240[5]+std_error_240[5], mean_error_240[6]+std_error_240[6], mean_error_240[7]+std_error_240[7], mean_error_240[8]+std_error_240[8]]

# 0480hz_00ms.txt NO FAILURE, mean=10.661285833565266, std=6.217748064935702
# 0480hz_05ms.txt NO FAILURE, mean=11.722541906429822, std=6.562974113576045
# 0480hz_10ms.txt NO FAILURE, mean=12.199688473520249, std=7.627442659429762
# 0480hz_15ms.txt NO FAILURE, mean=13.17506791427709, std=8.521637745946606
# 0480hz_20ms.txt NO FAILURE, mean=13.976016379058205, std=8.967720758473394
# 0480hz_25ms.txt NO FAILURE, mean=14.558522727272727, std=9.087046604892235
# 0480hz_30ms.txt NO FAILURE, mean=14.565856416054269, std=13.471838552683108
# 0480hz_35ms.txt NO FAILURE, mean=16.775014979029358, std=9.926241408452489
# 0480hz_40ms.txt NO FAILURE, mean=16.07186322804984, std=12.025233995937027
# 0480hz_45ms.txt FAILED AT 2.759316647876889 242.79042866863233
# 0480hz_50ms.txt FAILED AT 3.471404787542208 81.52998522895126
# 0480hz_25ms_2 NO FAILURE, mean=12.788374205267939, std=9.496004761974564
# 0480hz_30ms_2 NO FAILURE, mean=18.112352763515553, std=13.41060094440646
# 0480hz_40ms NO FAILURE, mean=18.143661971830987, std=12.952975042247749
# 0480hz_450ms 1.1951953097699421
# 0480hz_45ms 3.8380730695120526
# 0480hz_50ms 3.66824078821481

time_to_failure_480 = [10,10,10,10,10,10,10,10,10,3.14374298892979, 3.68786916592843]
mean_error_480 = [10.661285833565266, 11.722541906429822, 12.199688473520249, 13.17506791427709, 13.976016379058205, 14.558522727272727, 14.565856416054269, 16.775014979029358, 16.07186322804984, 20,20]
std_error_480 = [6.217748064935702, 6.562974113576045, 7.627442659429762, 8.521637745946606, 8.967720758473394, 9.087046604892235, 13.471838552683108, 9.926241408452489, 12.025233995937027, 0, 0]

diff_std_480 = [mean_error_480[0]-std_error_480[0], mean_error_480[1]-std_error_480[1], mean_error_480[2]-std_error_480[2], mean_error_480[3]-std_error_480[3], mean_error_480[4]-std_error_480[4], mean_error_480[5]-std_error_480[5], mean_error_480[6]-std_error_480[6], mean_error_480[7]-std_error_480[7], mean_error_480[8]-std_error_480[8]]
sum_std_480 = [mean_error_480[0]+std_error_480[0], mean_error_480[1]+std_error_480[1], mean_error_480[2]+std_error_480[2], mean_error_480[3]+std_error_480[3], mean_error_480[4]+std_error_480[4], mean_error_480[5]+std_error_480[5], mean_error_480[6]+std_error_480[6], mean_error_480[7]+std_error_480[7], mean_error_480[8]+std_error_480[8]]


# 0720hz_00ms.txt NO FAILURE, mean=16.88957528957529, std=10.74323740837329
# 0720hz_05ms.txt NO FAILURE, mean=16.94547053649956, std=10.986624120407328
# 0720hz_10ms.txt NO FAILURE, mean=20.64391951006124, std=12.626752808014862
# 0720hz_15ms.txt FAILED AT 6.037579527558583 93.84410646387833
# 0720hz_20ms.txt FAILED AT 5.065046411722738 133.4945215485756
# 0720hz_25ms.txt FAILED AT 9.260457109282228 53.15099078984092
# 0720hz_30ms.txt FAILED AT 6.619779402719601 179.48614775725594
# 0720hz_35ms.txt FAILED AT 3.8245479968376426 123.04777070063695
# 0720hz_40ms.txt FAILED AT 2.4044756971384516 203.7508064516129
# 0720hz_45ms.txt FAILED AT 2.1239183874642515 132.09404096834265
# 0720hz_50ms.txt FAILED AT 1.7209391527076967 132.18639798488664
# 0720hz_20ms 2.804336926306239
# 0720hz_20ms_2 5.512458900374963
# 0720hz_25ms NO FAILURE, mean=26.955057309471144, std=21.37398103334726
# 0720hz_45ms 2.008637848181193
# 0720hz_50ms 2.258546446229443

time_to_failure_720 = [10,10,10, 6.423315502289267, 5.257952817009892, 9.260457109282228, 6.710706830762485, 3.848629154649702, 2.2841280048713424, 1.2583379758951239, 1.4798809556466948]
mean_error_720 = [16.88957528957529, 16.94547053649956, 20.64391951006124, 30, 30, 30 ,30,30,30,30,30]
std_error_720 = [10.74323740837329, 10.986624120407328, 12.626752808014862, 0, 0, 0, 0, 0, 0, 0, 0]

diff_std_720 = [mean_error_720[0]-std_error_720[0], mean_error_720[1]-std_error_720[1], mean_error_720[2]-std_error_720[2]]
sum_std_720 = [mean_error_720[0]+std_error_720[0], mean_error_720[1]+std_error_720[1], mean_error_720[2]+std_error_720[2]]

# 0960hz_00ms.txt NO FAILURE, mean=19.05692031156381, std=15.511965898035307
# 0960hz_05ms.txt NO FAILURE, mean=23.39025804581038, std=15.036793693797707
# 0960hz_10ms.txt NO FAILURE, mean=9.670259248470725, std=12.507437152179522
# 0960hz_15ms.txt FAILED AT 2.1240648437282195 120.54584650761713
# 0960hz_20ms.txt FAILED AT 2.541293960134051 197.75379537953796
# 0960hz_25ms.txt FAILED AT 6.493877976377168 201.70533745037494
# 0960hz_30ms.txt FAILED AT 5.392248251456301 139.0306158775365
# 0960hz_35ms.txt FAILED AT 5.796752688214433 63.50995879120879
# 0960hz_40ms.txt FAILED AT 3.270814650852505 66.43479549496146
# 0960hz_45ms.txt FAILED AT 3.6255487083472797 60.84225941422594
# 0960hz_50ms.txt FAILED AT 2.607385218566124 276.10141020323516
# 0960hz_25ms 7.2783315861444144
# 0960hz_45ms 3.257228169017973

time_to_failure_960 = [10, 10, 4.4298994200665405, 2.1240648437282195, 2.609482711325846, 6.46980590431058, 5.567120304999471, 5.604314105886168, 3.0299696939905845, 3.483170206316413, 2.535317873564792]
mean_error_960 = [19.05692031156381, 23.39025804581038, 30, 30, 30, 30, 30, 30, 30, 30, 30]
std_error_960 = [15.511965898035307, 15.036793693797707, 0,0,0,0,0,0,0,0,0 ]

diff_std_960 = [mean_error_960[0]-std_error_960[0], mean_error_960[1]-std_error_960[1]]
sum_std_960 = [mean_error_960[0]+std_error_960[0], mean_error_960[1]+std_error_960[1]]

fig, ax1 = plt.subplots(figsize=(15, 6))
ax2 = ax1.twinx()
ax1.plot(latencies_1440[0:9], mean_error_1440[0:9], color='tab:blue', label='event camera')
ax1.plot(latencies_1440[10:13], mean_error_1440[10:13], color='white')
ax2.scatter(9,time_to_failure_1440[10], s=100,marker='x',color='tab:purple',linewidths=2)
ax2.scatter(10,time_to_failure_1440[11], s=100,marker='x',color='tab:purple',linewidths=2)
ax2.scatter(11,time_to_failure_1440[12], s=100,marker='x',color='tab:purple',linewidths=2)
ax1.set_xlabel('Added latencies [ms]')
ax1.set_ylabel('Mean error [px]', color='tab:blue')
ax2.set_ylabel('Time to failure [s]', color='tab:purple')
plt.title("Target speed = 1440 pix/s and Kp = 0.005")

# 1440Hz_00ms_0.005 NO FAILURE 59.96098149637972
# 1440hz_00ms_0.01 NO FAILURE 59.82451197424029
# 1440hz_05ms_0.005 NO FAILURE 59.906878519710375
# 1440hz_05ms_0.01 NO FAILURE 63.341912134311855
# 1440hz_10ms_0.005 NO FAILURE 62.15633802816902
# 1440hz_10ms_0.01 NO FAILURE 66.93700391841656
# 1440hz_15ms_0.005 NO FAILURE 60.40611178126257
# 1440hz_15ms_0.01 3.6587380288251703
# 1440hz_20ms_0.005 NO FAILURE 61.270699356913184
# 1440hz_20ms_0.01 0.917417604595018
# 1440hz_25ms_0.005 NO FAILURE 63.35974261009451
# 1440hz_25ms_0.01 1.2698965212562277
# 1440hz_30ms_0.005 NO FAILURE 65.10467571644043
# 1440hz_30ms_0.01 1.2457449834771221
# 1440hz_35ms NO FAILURE 67.49017236165709
# 1440hz_40ms_0.01 1.1317647719330544
# 1440hz_50ms NO FAILURE 72.41933862699769
# 1440hz_55ms NO FAILURE 75.39682220434433
# 1440hz_60ms 1.030989200684187
# 1440hz_60ms_2 8.387627173573163
# 1440hz_65ms 6.288995189622945
# 1440hz_70ms NO FAILURE 86.65457842248414
# 1440hz_70ms_2 NO FAILURE 87.22175226586103
# 1440hz_80ms 1.0765299476920749


latencies_4000 = ['0', '10', '20', '30', '50']

mean_error_4000 = [114.27716408917453,145.8442861442761, 150,150,150]
time_to_failure_4000 = [10,10, 5.589231455796803,3.393448236599828,0.9754423222876207]

fig, ax1 = plt.subplots(figsize=(15, 6))
ax2 = ax1.twinx()
ax1.plot(latencies_4000[0:2], mean_error_4000[0:2], color='tab:blue', label='event camera')
ax1.plot(latencies_4000[2:5], mean_error_4000[2:5], color='white')
ax2.scatter(2,time_to_failure_4000[2], s=100,marker='x',color='tab:purple',linewidths=2)
ax2.scatter(3,time_to_failure_4000[3], s=100,marker='x',color='tab:purple',linewidths=2)
ax2.scatter(4,time_to_failure_4000[4], s=100,marker='x',color='tab:purple',linewidths=2)
ax1.set_xlabel('Added latencies [ms]')
ax1.set_ylabel('Mean error [px]', color='tab:blue')
ax2.set_ylabel('Time to failure [s]', color='tab:purple')
plt.title("Target speed = 4000 pix/s and Kp = 0.005")

# 4000hz_00ms NO FAILURE 114.27716408917453
# 4000hz_00ms_2 NO FAILURE 142.69376257545272
# 4000hz_10ms NO FAILURE 145.8442861442761
# 4000hz_20ms 5.589231455796803
# 4000hz_30ms 3.393448236599828
# 4000hz_50ms 0.9754423222876207
# 4000hz_60ms 1.2233634197114949


fig, ax1 = plt.subplots(figsize=(15, 8))
# ax2 = ax1.twinx()
ax1.plot(latencies_240[0:9], mean_error_240[0:9], color='tab:purple', label='V1')
plt.fill_between(latencies_240[0:9], diff_std_240, sum_std_240, alpha=0.15, color="tab:purple")
ax1.plot(latencies_240[9:11], mean_error_240[9:11], color='white')

ax1.plot(latencies_240[0:9], mean_error_480[0:9], color='tab:orange', label='V2')
plt.fill_between(latencies_240[0:9], diff_std_480, sum_std_480, alpha=0.15, color="tab:orange")
ax1.plot(latencies_240[9:11], mean_error_480[9:11], color='white')

ax1.plot(latencies_240[0:3], mean_error_720[0:3], color='tab:red', label='V3')
plt.fill_between(latencies_240[0:3], diff_std_720, sum_std_720, alpha=0.15, color="tab:red")
ax1.plot(latencies_240[3:4], mean_error_720[3:4], color='white')
ax1.plot(latencies_240[6:11], mean_error_720[6:11], color='white')

ax1.plot(latencies_240[0:2], mean_error_960[0:2], color='tab:green', label='V4')
plt.fill_between(latencies_240[0:2], diff_std_960, sum_std_960, alpha=0.15, color="tab:green")
ax1.plot(latencies_240[2:4], mean_error_960[2:4], color='white')
ax1.plot(latencies_240[6:11], mean_error_960[6:11], color='white')

# ax1.axvline(x = 60, color = 'k', label = 'axvline - full height')

ax1.scatter(8, mean_error_240[8], s=100,marker='x',color='tab:purple',linewidths=2)
ax1.scatter(8, mean_error_480[8], s=100,marker='x',color='tab:orange',linewidths=2)
ax1.scatter(2, mean_error_720[2], s=100,marker='x',color='tab:red',linewidths=2)
ax1.scatter(1, mean_error_960[1], s=100,marker='x',color='tab:green',linewidths=2)

ax1.axvline(x = 7, color='k', ls = '--', label='real-sense\nlatency')

# ax2.scatter(9,time_to_failure_240[9], s=100,marker='x',color='tab:purple',linewidths=2)
# ax2.scatter(10,time_to_failure_240[10], s=100,marker='x',color='tab:purple',linewidths=2)

# ax2.scatter(9,time_to_failure_480[9], s=100,marker='x',color='tab:orange',linewidths=2)
# ax2.scatter(10,time_to_failure_480[10], s=100,marker='x',color='tab:orange',linewidths=2)

# ax2.scatter(3,time_to_failure_720[3], s=100,marker='x',color='tab:red',linewidths=2)
# ax2.scatter(4,time_to_failure_720[4], s=100,marker='x',color='tab:red',linewidths=2)
# ax2.scatter(5,time_to_failure_720[5], s=100,marker='.',color='tab:red',linewidths=2)
# ax2.scatter(6,time_to_failure_720[6], s=100,marker='x',color='tab:red',linewidths=2)
# ax2.scatter(7,time_to_failure_720[7], s=100,marker='x',color='tab:red',linewidths=2)
# ax2.scatter(8,time_to_failure_720[8], s=100,marker='x',color='tab:red',linewidths=2)
# ax2.scatter(9,time_to_failure_720[9], s=100,marker='x',color='tab:red',linewidths=2)
# ax2.scatter(10,time_to_failure_720[10], s=100,marker='x',color='tab:red',linewidths=2)

# ax2.scatter(2,time_to_failure_960[2], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(3,time_to_failure_960[3], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(4,time_to_failure_960[4], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(5,time_to_failure_960[5], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(6,time_to_failure_960[6], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(7,time_to_failure_960[7], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(8,time_to_failure_960[8], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(9,time_to_failure_960[9], s=100,marker='x',color='tab:green',linewidths=2)
# ax2.scatter(10,time_to_failure_960[10], s=100,marker='x',color='tab:green',linewidths=2)

ax1.set_xlabel('Added latencies [ms]')
ax1.set_ylabel('Mean error [px]', color='k')
ax2.set_ylabel('Time to failure [s]', color='k')
ax1.legend(loc='upper right')
plt.show()