import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from pylab import *
import random



def splitXYZarray(arr):

    X = []
    Y = []
    Z = []
    W = []

    for i in arr:
        X.append(i[0])
        Y.append(i[1])
        Z.append(i[2])
        W.append(i[3])

    return [X, Y, Z, W]


# 60 verts 20 polys
T1 = np.array([
[5.097792,
5.042016,
5.231456,
5.118495,
5.093568,
5.162271,
5.157056,
5.119904,
5.299104,
5.127264], 
[5.132032,
5.091008,
5.220544,
5.282880,
5.138560,
5.470655,
5.159328,
5.146272,
5.091776,
5.099008], 
[25.096449,
25.017120,
24.939074,
25.039646,
24.983360,
24.947840,
25.120129,
24.955841,
25.008287,
25.100609], 
[25.421089,
25.262560,
25.438911,
25.636898,
25.391617,
25.553825,
25.397953,
25.505856,
25.285471,
25.405121]
])



# 240 verts 80 polys
T2 = np.array([
[7.337888,
7.065088,
7.328832,
7.385088,
7.115936,
7.083104,
7.359104,
7.283616,
7.349504,
7.349696],
[7.095232,
7.023680,
7.293664,
7.143520,
6.968161,
7.063616,
6.975168,
7.176864,
7.066528,
7.158816],
[26.902014,
26.663681,
27.131744,
26.963392,
26.855425,
26.813663,
26.898914,
26.902657,
26.794462,
27.070593],
[27.196800,
26.889088,
27.145920,
27.018978,
27.155966,
26.913952,
26.986689,
27.299583,
26.993183,
27.204416]
])



# 540 verts 180 polys
T3 = np.array([
[13.093504,
13.119072,
12.638752,
13.161472,
12.964448,
12.840448,
13.174687,
13.261407,
13.248544,
13.138400],
[12.783552,
13.172832,
13.200577,
13.094080,
13.269441,
13.448800,
12.847264,
13.324256,
12.987583,
13.310721],
[27.792736,
27.647392,
27.911520,
27.917921,
27.905697,
27.957249,
28.231390,
27.953762,
27.834688,
28.019360],
[28.157471,
28.257153,
28.299423,
28.302656,
28.330370,
28.441919,
28.262079,
28.013472,
28.237825,
28.216866]
])



# 960 verts 320 polys
T4 = np.array([
[19.194176,
19.095999,
18.971487,
19.285664,
19.330751,
19.724289,
18.962431,
19.195553,
19.206528,
18.405025],
[18.937216,
18.959646,
18.650177,
19.033377,
19.378176,
19.131392,
19.487808,
18.769089,
19.184513,
18.890240],
[28.353729,
28.618721,
28.729729,
28.801983,
28.470272,
28.827936,
28.956223,
28.613600,
28.682335,
28.666336],
[28.718048,
29.021278,
28.820450,
28.835840,
28.926399,
28.973503,
28.940960,
29.013662,
28.919615,
28.870142]
])



# 1.5k verts 500 polys
T5 = np.array([
[26.900320,
26.678883,
26.410433,
26.791136,
27.221504,
27.097857,
26.694145,
26.592033,
26.635006,
27.475744],
[26.847519,
27.260832,
26.759071,
26.091297,
26.599712,
26.650658,
26.529472,
26.673281,
26.083231,
26.636385],
[30.147423,
29.954977,
30.128767,
30.171871,
29.952223,
30.412449,
30.285601,
29.945570,
29.974752,
30.307617],
[29.748001,
29.797407,
29.823330,
29.752001,
29.732544,
29.734783,
29.861599,
29.507360,
29.305634,
29.956480]
])



# 2.16k verts 720 polys
T6 = np.array([
[37.877502,
37.489697,
37.831009,
37.370945,
37.518497,
38.172607,
37.944897,
38.029247,
37.516129,
37.391327],
[36.778084,
37.609760,
36.161247,
36.646271,
36.347332,
37.048134,
37.310848,
36.659103,
36.734306,
37.186817],
[31.032320,
30.588127,
30.847233,
31.127777,
30.858335,
30.931072,
30.938208,
30.982080,
30.711040,
31.127520],
[30.422783,
30.408993,
30.758625,
30.486977,
30.350784,
30.468065,
30.281406,
30.366720,
30.165855,
30.268511]
])



# 2.94k verts 980 polys
T7 = np.array([
[49.787037,
50.114559,
49.345249,
49.262814,
49.864002,
50.427105,
49.986244,
50.269180,
49.790756,
49.324677],
[47.693634,
48.184158,
48.587425,
48.156673,
47.703808,
48.551712,
48.507809,
48.366432,
48.267746,
48.106884],
[31.493568,
31.425730,
31.997025,
31.696127,
31.136736,
31.581089,
31.440386,
31.779264,
31.649824,
31.783745],
[30.561888,
30.627296,
31.211552,
30.777218,
30.754814,
30.780064,
31.078337,
30.798113,
30.843136,
30.835041]
])



# 3.84k verts 1.28k polys
T8 = np.array([
[61.709564,
62.124512,
62.026787,
61.804031,
61.951645,
62.010300,
62.162239,
61.901474,
62.363644,
61.973728],
[60.414787,
60.626019,
60.677120,
60.349346,
60.122787,
60.167652,
60.213627,
60.297665,
60.267391,
60.039967],
[32.375523,
32.330017,
32.590305,
32.605343,
32.148445,
32.365406,
32.354240,
32.267902,
32.134209,
32.638145],
[31.399233,
31.533442,
31.459490,
31.447041,
31.310623,
31.253313,
31.205248,
31.056959,
31.109503,
31.591009]
])




test1 = np.array([[5.097792, 5.132032, 25.096449, 25.421089],
[5.042016, 5.091008, 25.017120, 25.262560],
[5.231456, 5.220544, 24.939074, 25.438911],
[5.118495, 5.282880, 25.039646, 25.636898],
[5.093568, 5.138560, 24.983360, 25.391617],
[5.162271, 5.470655, 24.947840, 25.553825],
[5.157056, 5.159328, 25.120129, 25.397953],
[5.119904, 5.146272, 24.955841, 25.505856],
[5.299104, 5.091776, 25.008287, 25.285471],
[5.127264, 5.099008, 25.100609, 25.405121]])


test2 = np.array([[7.337888, 7.095232, 26.902014, 27.196800],
[7.065088, 7.023680, 26.663681, 26.889088],
[7.328832, 7.293664, 27.131744, 27.145920],
[7.385088, 7.143520, 26.963392, 27.018978],
[7.115936, 6.968161, 26.855425, 27.155966],
[7.083104, 7.063616, 26.813663, 26.913952],
[7.359104, 6.975168, 26.898914, 26.986689],
[7.283616, 7.176864, 26.902657, 27.299583],
[7.349504, 7.066528, 26.794462, 26.993183],
[7.349696, 7.158816, 27.070593, 27.204416]])


test3 = np.array([[13.093504, 12.783552, 27.792736, 28.157471],
[13.119072, 13.172832, 27.647392, 28.257153],
[12.638752, 13.200577, 27.911520, 28.299423],
[13.161472, 13.094080, 27.917921, 28.302656],
[12.964448, 13.269441, 27.905697, 28.330370],
[12.840448, 13.448800, 27.957249, 28.441919],
[13.174687, 12.847264, 28.231390, 28.262079],
[13.261407, 13.324256, 27.953762, 28.013472],
[13.248544, 12.987583, 27.834688, 28.237825],
[13.138400, 13.310721, 28.019360, 28.216866]])


test4 = np.array([[19.194176, 18.937216, 28.353729, 28.718048],
[19.095999, 18.959646, 28.618721, 29.021278],
[18.971487, 18.650177, 28.729729, 28.820450],
[19.285664, 19.033377, 28.801983, 28.835840],
[19.330751, 19.378176, 28.470272, 28.926399],
[19.724289, 19.131392, 28.827936, 28.973503],
[18.962431, 19.487808, 28.956223, 28.940960],
[19.195553, 18.769089, 28.613600, 29.013662],
[19.206528, 19.184513, 28.682335, 28.919615],
[18.405025, 18.890240, 28.666336, 28.870142]])


test5 = np.array([[26.900320, 26.847519, 30.147423, 29.748001],
[26.678883, 27.260832, 29.954977, 29.797407],
[26.410433, 26.759071, 30.128767, 29.823330],
[26.791136, 26.091297, 30.171871, 29.752001],
[27.221504, 26.599712, 29.952223, 29.732544],
[27.097857, 26.650658, 30.412449, 29.734783],
[26.694145, 26.529472, 30.285601, 29.861599],
[26.592033, 26.673281, 29.945570, 29.507360],
[26.635006, 26.083231, 29.974752, 29.305634],
[27.475744, 26.636385, 30.307617, 29.956480]])


test6 = np.array([[37.877502, 36.778084, 31.032320, 30.422783],
[37.489697, 37.609760, 30.588127, 30.408993],
[37.831009, 36.161247, 30.847233, 30.758625],
[37.370945, 36.646271, 31.127777, 30.486977],
[37.518497, 36.347332, 30.858335, 30.350784],
[38.172607, 37.048134, 30.931072, 30.468065],
[37.944897, 37.310848, 30.938208, 30.281406],
[38.029247, 36.659103, 30.982080, 30.366720],
[37.516129, 36.734306, 30.711040, 30.165855],
[37.391327, 37.186817, 31.127520, 30.268511]])


test7 = np.array([[49.787037, 47.693634, 31.493568, 30.561888],
[50.114559, 48.184158, 31.425730, 30.627296],
[49.345249, 48.587425, 31.997025, 31.211552],
[49.262814, 48.156673, 31.696127, 30.777218],
[49.864002, 47.703808, 31.136736, 30.754814],
[50.427105, 48.551712, 31.581089, 30.780064],
[49.986244, 48.507809, 31.440386, 31.078337],
[50.269180, 48.366432, 31.779264, 30.798113],
[49.790756, 48.267746, 31.649824, 30.843136],
[49.324677, 48.106884, 31.783745, 30.835041]])


test8 = np.array([[61.709564, 60.414787, 32.375523, 31.399233],
[62.124512, 60.626019, 32.330017, 31.533442],
[62.026787, 60.677120, 32.590305, 31.459490],
[61.804031, 60.349346, 32.605343, 31.447041],
[61.951645, 60.122787, 32.148445, 31.310623],
[62.010300, 60.167652, 32.365406, 31.253313],
[62.162239, 60.213627, 32.354240, 31.205248],
[61.901474, 60.297665, 32.267902, 31.056959],
[62.363644, 60.267391, 32.134209, 31.109503],
[61.973728, 60.039967, 32.638145, 31.591009]])


testnames = ['bruteforce', 'bounding box', 'kd-tree', 'short-stack kd-tree']
namedict = {0:testnames[0],
            1:testnames[1],
            2:testnames[2],
            3:testnames[3]}

testnamesdict = ['3k\nvertices',
                 '4.5k\nvertices',
                 '9k\nvertices',
                 '18.4k\nvertices',
                 '37.5k\nvertices',
                 '75k\nvertices',
                 '150k\nvertices',
                 '300k\nvertices']

testnames = ['caching_off',
             'caching_on',
             'no_stream_compaction',
             'sorting_off',
             'blocksize_16',
             'blocksize_32',
             'blocksize_64',
             'blocksize_128',
             'blocksize_256']

def autolabel(rects):
    # attach some text labels
    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                '%.0f' % height,
                ha='center', va='bottom')




tests = [test1, test2, test3, test4, test5, test6, test7, test8]

for i in range(len(tests)):
    tests[i] = splitXYZarray(tests[i])

#print test1


matplotlib.rc("figure", facecolor="black")

xaxis = range(1, 11)
fig = plt.figure(facecolor='black')
#fig.patch.set_alpha(1.0)
fig.patch.set_facecolor('black')
plt.rcParams['text.color'] = 'gray'
plt.rcParams['axes.facecolor'] = 'black'
plt.rcParams['lines.linewidth'] = 4

'''
fig = plt.figure(facecolor='black')
fig.patch.set_alpha(0.0)
fig.patch.set_facecolor('black')
fig.patch.set_color('white')
plt.rcParams['text.color'] = 'white'
plt.rcParams['axes.facecolor'] = 'black'
'''
r = 0.0
g = 0.0
b = 1.0
testnum = 8
'''
ax = fig.add_subplot(111)
for i in range(testnum):
    testname = testnamesdict[i]
    
    g1 = random.random()
    g2 = random.random()
    g3 = random.random()
    
    #ax = fig.add_subplot(111)

    ax.set_title(namedict[1])
    ax.plot(xaxis, tests[i][1], color=[1.0*i/testnum, b, 1.0-(1.0*i/testnum)])
    ax.text(68.0, 3.0 + i*3.0/testnum, testnames[i], color=[1.0*i/testnum, g2, 1.0-(1.0*i/testnum)])

    ax.plot(xaxis, tests[i][2], color=[1.0*i/testnum, 0, 1.0-(1.0*i/testnum)])
    ax.set_xlabel('iterations') 
    ax.set_ylabel('time') 
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.tick_params(axis='x', colors='white')
    ax.tick_params(axis='y', colors='white')
    ax.axis((1,100,0,10))
    plt.axis((1,101,0,10))
    fig.savefig('deleteme_%s.png' % testnames[i], transparent=True)
    #plt.show()
    #fig.delaxes(ax)
'''
#plt.show()
#exit()

#plt.axis((1,101,0,10))
fig.savefig('all.png')
#plt.show()

#exit()
plt.rcParams['legend.loc'] = 'upper left'

p_bruteforce = []
p_bbox = []
p_kd = []
p_kdshortstack = []


for i in range(testnum):
    p_bruteforce.append(np.average(tests[i][0]))
    p_bbox.append(np.average(tests[i][1]))
    p_kd.append(np.average(tests[i][2]))
    p_kdshortstack.append(np.average(tests[i][3]))



fig, ax = plt.subplots()
genray = ax.bar(range(0,testnum), p_bruteforce, align='center', alpha=0.5)
trace = ax.bar(range(0,testnum), p_bbox, align='center', alpha=0.5)
shade = ax.bar(range(0,testnum), p_kd, align='center', alpha=0.5)
kdshort = ax.bar(range(0,testnum), p_kdshortstack, align='center', alpha=0.5)
#plt.xticks(range(0,testnum), testnamesdict)


width = 0.25      # the width of the bars
xaxis = np.arange(1.0, testnum+1)

fig, ax = plt.subplots()
rects1 = ax.bar(xaxis, p_bruteforce, width, color='#995522')
rects2 = ax.bar(xaxis+width*1.0, p_bbox, width, color='#882255')
rects3 = ax.bar(xaxis+width*2.0, p_kd, width, color='#118888')
rects4 = ax.bar(xaxis+width*3.0, p_kdshortstack, width, color='#1166cc')
rects5 = ax.bar(xaxis+width*4.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], width, color='#1166cc')

fig2 = plt.figure(facecolor='black')
fig.patch.set_facecolor('black')
ax.patch.set_facecolor('black')

# add some text for labels, title and axes ticks
ax.set_ylabel('Test Benchmark Averages')
ax.set_title('Average time')
ax.set_xticks(xaxis+width)
ax.set_xticklabels(testnamesdict)
ax.xaxis.label.set_color('white')
ax.yaxis.label.set_color('white')
ax.tick_params(axis='x', colors='white')
ax.tick_params(axis='y', colors='white')

ax.legend((rects1[0], rects2[0], rects3[0], rects4[0]), (namedict[0], namedict[1], namedict[2], namedict[3]))

autolabel(rects1)
autolabel(rects2)
autolabel(rects3)
autolabel(rects4)

ax.axis((1,9,0,65))


fig.savefig('benchmark_results_low1.png', transparent=True)


'''
# show shading graph which is invisible at the moment
fig, ax = plt.subplots()
rects3 = ax.bar(xaxis, p_shadeMaterial, width, color='#1122ff')#, yerr=p_pathTraceOneBounce)

fig = plt.figure(facecolor='black')
fig.patch.set_facecolor('black')
ax.patch.set_facecolor('black')

# add some text for labels, title and axes ticks
ax.set_ylabel('Test Benchmark Averages')
ax.set_title('Average time')
ax.set_xticks(xaxis)
ax.set_xticklabels(testnamesdict)
ax.xaxis.label.set_color('white')
ax.yaxis.label.set_color('white')
ax.tick_params(axis='x', colors='white')
ax.tick_params(axis='y', colors='white')

ax.legend([rects3[0]], [namedict[2]])


autolabel(rects3)
ax.axis((1,10,0,0.05))
plt.show()
'''

plt.show()

#fig.delaxes(ax)
fig, ax = plt.subplots()
ax.axis((1,8,1,70))

plt.rcParams['lines.linewidth'] = 1

for i in range(4):
    #fig, ax = plt.subplots()
    ax.set_ylabel('Test Benchmark Averages')
    ax.set_title('Average time')
    color = '#990000'
    ax.set_xticks(xaxis)
    ax.set_xticklabels(['60', '240', '540', '960', '1.5k', '2.16k', '2.94k', '3.84k'])
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.tick_params(axis='x', colors='white')
    ax.tick_params(axis='y', colors='white')

    ax.plot([0.0,
             T1[i].mean(), 
             T2[i].mean(), 
             T3[i].mean(), 
             T4[i].mean(), 
             T5[i].mean(), 
             T6[i].mean(), 
             T7[i].mean(), 
             T8[i].mean()])
    plt.xlabel('vertices (x1000)')
    plt.ylabel('time in ms')
    #plt.savefig('deleteme%d.png' % i, transparent=True)




legend(['bruteforce', 'bounding box', 'kd-tree', 'short-stack kd-tree'])
fig.savefig('benchmark_results_low2.png', transparent=True)

#plt.show()




