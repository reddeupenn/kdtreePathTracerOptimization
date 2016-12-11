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


# 3k verts 1k polys
T1 = np.array([
[53.452290,
53.342525,
53.728256,
53.464287,
53.717186,
53.479683,
53.563648,
53.363300,
53.492348,
53.527489], 
[49.272064,
49.222210,
49.484932,
49.302368,
49.206203,
49.136894,
49.361565,
49.186783,
49.367519,
49.231102], 
[51.599041,
51.333378,
51.795837,
52.001152,
51.961155,
51.565025,
51.504547,
51.538589,
52.097538,
51.967682], 
[41.238239,
40.039585,
41.268188,
40.854973,
40.770596,
40.837376,
40.490593,
41.069954,
40.588032,
41.132641]
])



# 4.5k verts 1.5k polys
T2 = np.array([
[78.853249,
78.936737,
79.048477,
79.196037,
78.925568,
79.035522,
79.050240,
79.042618,
78.956474,
79.035103],
[72.754364,
72.921753,
73.034882,
72.940826,
72.817818,
72.919998,
72.760834,
73.086136,
72.923492,
73.065025],
[56.752930,
56.241920,
56.759136,
56.665955,
56.830082,
56.125153,
56.374622,
56.896160,
56.390976,
57.168766],
[43.924160,
43.218620,
43.839615,
44.169857,
44.133667,
44.419807,
44.438622,
43.818302,
43.797508,
44.327679]
])



# 9k verts 3k polys
T3 = np.array([
[155.846344,
155.825073,
156.267471,
156.124802,
156.016357,
155.924194,
155.953827,
156.139496,
156.066147,
156.003845],
[143.690872,
143.762238,
143.715240,
144.248718,
143.864029,
143.771561,
143.872665,
143.876892,
143.859833,
144.101517],
[68.146011,
67.882370,
69.242523,
69.118752,
70.088188,
69.018112,
68.828575,
68.742561,
69.988609,
69.601181],
[49.671108,
48.514397,
49.765507,
49.468197,
50.534496,
50.287521,
49.283905,
49.527359,
49.601055,
50.125374]
])


# 18.372k verts 6.124k polys
T4 = np.array([
[315.254059,
315.306122,
315.429718,
315.527161,
315.162170,
315.521698,
315.409149,
315.567413,
315.161407,
315.314362],
[295.667053,
291.646729,
295.767517,
295.698090,
295.399170,
291.961884,
291.520813,
295.591370,
295.286530,
295.972473],
[93.522079,
91.767975,
93.592896,
95.368385,
95.915260,
95.745148,
94.456963,
94.761543,
94.372513,
92.888199],
[58.779938,
57.549667,
59.016479,
59.534847,
60.660225,
59.230145,
58.782433,
58.822624,
58.532829,
59.001152]
])

#37.5k verts 12.5k polys
T5 = np.array([
[641.841797,
641.546936,
642.534424,
642.145691,
642.129150,
642.068481,
642.421631,
641.961609,
642.166626,
642.689575],
[592.676941,
593.308105,
592.676880,
593.097473,
592.659180,
593.411133,
593.182678,
592.932861,
592.748718,
592.885986],
[134.830750,
130.970718,
137.645401,
137.855225,
137.480988,
135.646088,
137.738617,
137.178757,
136.812408,
137.911423],
[79.795837,
77.705307,
78.838364,
78.963196,
81.677467,
78.810333,
79.987297,
78.317093,
79.505447,
79.929176]
])

# 75k verts 25k polys
T6 = np.array([
[2281.165283,
2280.900879,
2281.699951,
2280.524414,
2281.018555,
2280.698730,
2280.429688,
2280.663086,
2280.578369,
2281.381104],
[2180.721680,
2180.258545,
2182.276123,
2183.135010,
2181.421875,
2181.740723,
2181.965332,
2181.996338,
2180.654297,
2181.621582],
[212.943924,
208.482117,
215.839905,
218.825912,
220.689209,
218.186066,
214.664719,
216.239044,
216.630615,
216.799942],
[116.712990,
113.440872,
118.723839,
119.403831,
121.430122,
122.408318,
119.276039,
114.977180,
118.388641,
118.700966]
])

# 50k verts 150k polys
T7 = np.array([
[3556.819092,
3557.768799,
3558.853760,
3557.284424,
3558.567139,
3557.649414,
3558.996582,
3557.598633,
3557.660400,
3557.803955],
[3361.569336,
3360.592285,
3361.468262,
3360.384521,
3359.667480,
3358.281006,
3357.858643,
3362.512451,
3359.337158,
3361.268311],
[378.477875,
366.524597,
389.975555,
389.518127,
389.575409,
377.177643,
382.524841,
389.808594,
381.883789,
378.545288],
[194.881500,
191.307190,
196.996017,
196.972275,
198.127502,
205.304962,
196.306854,
190.609055,
194.983322,
194.990631]
])

# 300k verts 150k polys
T8 = np.array([
[0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0],
[0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0,
0.0],
[696.824219,
675.081726,
694.614441,
707.251953,
713.409607,
704.823975,
701.153381,
702.913025,
704.176758,
694.627502],
[337.702118,
320.391083,
344.573364,
343.722870,
351.965515,
353.936035,
344.329956,
349.185974,
332.633698,
345.728424]
])




test1 = np.array([[53.452290, 49.272064, 51.599041, 41.238239],
[53.342525, 49.222210, 51.333378, 40.039585],
[53.728256, 49.484932, 51.795837, 41.268188],
[53.464287, 49.302368, 52.001152, 40.854973],
[53.717186, 49.206203, 51.961155, 40.770596],
[53.479683, 49.136894, 51.565025, 40.837376],
[53.563648, 49.361565, 51.504547, 40.490593],
[53.363300, 49.186783, 51.538589, 41.069954],
[53.492348, 49.367519, 52.097538, 40.588032],
[53.527489, 49.231102, 51.967682, 41.132641]])


test2 = np.array([[78.853249, 72.754364, 56.752930, 43.924160],
[78.936737, 72.921753, 56.241920, 43.218620],
[79.048477, 73.034882, 56.759136, 43.839615],
[79.196037, 72.940826, 56.665955, 44.169857],
[78.925568, 72.817818, 56.830082, 44.133667],
[79.035522, 72.919998, 56.125153, 44.419807],
[79.050240, 72.760834, 56.374622, 44.438622],
[79.042618, 73.086136, 56.896160, 43.818302],
[78.956474, 72.923492, 56.390976, 43.797508],
[79.035103, 73.065025, 57.168766, 44.327679]])


test3 = np.array([[155.846344, 143.690872, 68.146011, 49.671108],
[155.825073, 143.762238, 67.882370, 48.514397],
[156.267471, 143.715240, 69.242523, 49.765507],
[156.124802, 144.248718, 69.118752, 49.468197],
[156.016357, 143.864029, 70.088188, 50.534496],
[155.924194, 143.771561, 69.018112, 50.287521],
[155.953827, 143.872665, 68.828575, 49.283905],
[156.139496, 143.876892, 68.742561, 49.527359],
[156.066147, 143.859833, 69.988609, 49.601055],
[156.003845, 144.101517, 69.601181, 50.125374]])


test4 = np.array([[315.254059, 295.667053, 93.522079, 58.779938],
[315.306122, 291.646729, 91.767975, 57.549667],
[315.429718, 295.767517, 93.592896, 59.016479],
[315.527161, 295.698090, 95.368385, 59.534847],
[315.162170, 295.399170, 95.915260, 60.660225],
[315.521698, 291.961884, 95.745148, 59.230145],
[315.409149, 291.520813, 94.456963, 58.782433],
[315.567413, 295.591370, 94.761543, 58.822624],
[315.161407, 295.286530, 94.372513, 58.532829],
[315.314362, 295.972473, 92.888199, 59.001152]])


test5 = np.array([[641.841797, 592.676941, 134.830750, 79.795837],
[641.546936, 593.308105, 130.970718, 77.705307],
[642.534424, 592.676880, 137.645401, 78.838364],
[642.145691, 593.097473, 137.855225, 78.963196],
[642.129150, 592.659180, 137.480988, 81.677467],
[642.068481, 593.411133, 135.646088, 78.810333],
[642.421631, 593.182678, 137.738617, 79.987297],
[641.961609, 592.932861, 137.178757, 78.317093],
[642.166626, 592.748718, 136.812408, 79.505447],
[642.689575, 592.885986, 137.911423, 79.929176]])


test6 = np.array([[2281.165283, 2180.721680, 212.943924, 116.712990],
[2280.900879, 2180.258545, 208.482117, 113.440872],
[2281.699951, 2182.276123, 215.839905, 118.723839],
[2280.524414, 2183.135010, 218.825912, 119.403831],
[2281.018555, 2181.421875, 220.689209, 121.430122],
[2280.698730, 2181.740723, 218.186066, 122.408318],
[2280.429688, 2181.965332, 214.664719, 119.276039],
[2280.663086, 2181.996338, 216.239044, 114.977180],
[2280.578369, 2180.654297, 216.630615, 118.388641],
[2281.381104, 2181.621582, 216.799942, 118.700966]])


test7 = np.array([[3556.819092, 3361.569336, 378.477875, 194.881500],
[3557.768799, 3360.592285, 366.524597, 191.307190],
[3558.853760, 3361.468262, 389.975555, 196.996017],
[3557.284424, 3360.384521, 389.518127, 196.972275],
[3558.567139, 3359.667480, 389.575409, 198.127502],
[3557.649414, 3358.281006, 377.177643, 205.304962],
[3558.996582, 3357.858643, 382.524841, 196.306854],
[3557.598633, 3362.512451, 389.808594, 190.609055],
[3557.660400, 3359.337158, 381.883789, 194.983322],
[3557.803955, 3361.268311, 378.545288, 194.990631]])


test8 = np.array([[0.0, 0.0, 696.824219, 337.702118],
[0.0, 0.0, 675.081726, 320.391083],
[0.0, 0.0, 694.614441, 344.573364],
[0.0, 0.0, 707.251953, 343.722870],
[0.0, 0.0, 713.409607, 351.965515],
[0.0, 0.0, 704.823975, 353.936035],
[0.0, 0.0, 701.153381, 344.329956],
[0.0, 0.0, 702.913025, 349.185974],
[0.0, 0.0, 704.176758, 332.633698],
[0.0, 0.0, 694.627502, 345.728424]])


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
#fig.savefig('all.png')
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

ax.axis((1,9,0,3000))


fig.savefig('benchmark_results1.png', transparent=True)


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

#fig.delaxes(ax)
fig, ax = plt.subplots()
plt.rcParams['lines.linewidth'] = 1

for i in range(4):
    #fig, ax = plt.subplots()
    ax.set_ylabel('Test Benchmark Averages')
    ax.set_title('Average time')
    color = '#990000'
    ax.set_xticks(xaxis)
    ax.set_xticklabels(['3k', '4.5k', '9k', '18.4k', '37.5k', '75k', '150k', '300k'])
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



legend(['bruteforce', 'bounding box', 'kd-tree', 'short-stack kd-tree'])
fig.savefig('benchmark_results2.png', transparent=True)

#plt.show()




