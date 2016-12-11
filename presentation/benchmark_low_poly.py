import matplotlib.pyplot as plt
import numpy as np

p1 = [[5.097792, 5.132032, 25.096449, 25.421089],
[5.042016, 5.091008, 25.017120, 25.262560],
[5.231456, 5.220544, 24.939074, 25.438911],
[5.118495, 5.282880, 25.039646, 25.636898],
[5.093568, 5.138560, 24.983360, 25.391617],
[5.162271, 5.470655, 24.947840, 25.553825],
[5.157056, 5.159328, 25.120129, 25.397953],
[5.119904, 5.146272, 24.955841, 25.505856],
[5.299104, 5.091776, 25.008287, 25.285471],
[5.127264, 5.099008, 25.100609, 25.405121]]


p2 = [[7.337888, 7.095232, 26.902014, 27.196800],
[7.065088, 7.023680, 26.663681, 26.889088],
[7.328832, 7.293664, 27.131744, 27.145920],
[7.385088, 7.143520, 26.963392, 27.018978],
[7.115936, 6.968161, 26.855425, 27.155966],
[7.083104, 7.063616, 26.813663, 26.913952],
[7.359104, 6.975168, 26.898914, 26.986689],
[7.283616, 7.176864, 26.902657, 27.299583],
[7.349504, 7.066528, 26.794462, 26.993183],
[7.349696, 7.158816, 27.070593, 27.204416]]


p3 = [[13.093504, 12.783552, 27.792736, 28.157471],
[13.119072, 13.172832, 27.647392, 28.257153],
[12.638752, 13.200577, 27.911520, 28.299423],
[13.161472, 13.094080, 27.917921, 28.302656],
[12.964448, 13.269441, 27.905697, 28.330370],
[12.840448, 13.448800, 27.957249, 28.441919],
[13.174687, 12.847264, 28.231390, 28.262079],
[13.261407, 13.324256, 27.953762, 28.013472],
[13.248544, 12.987583, 27.834688, 28.237825],
[13.138400, 13.310721, 28.019360, 28.216866]]


p4 = [[19.194176, 18.937216, 28.353729, 28.718048],
[19.095999, 18.959646, 28.618721, 29.021278],
[18.971487, 18.650177, 28.729729, 28.820450],
[19.285664, 19.033377, 28.801983, 28.835840],
[19.330751, 19.378176, 28.470272, 28.926399],
[19.724289, 19.131392, 28.827936, 28.973503],
[18.962431, 19.487808, 28.956223, 28.940960],
[19.195553, 18.769089, 28.613600, 29.013662],
[19.206528, 19.184513, 28.682335, 28.919615],
[18.405025, 18.890240, 28.666336, 28.870142]]


p5 = [[26.900320, 26.847519, 30.147423, 29.748001],
[26.678883, 27.260832, 29.954977, 29.797407],
[26.410433, 26.759071, 30.128767, 29.823330],
[26.791136, 26.091297, 30.171871, 29.752001],
[27.221504, 26.599712, 29.952223, 29.732544],
[27.097857, 26.650658, 30.412449, 29.734783],
[26.694145, 26.529472, 30.285601, 29.861599],
[26.592033, 26.673281, 29.945570, 29.507360],
[26.635006, 26.083231, 29.974752, 29.305634],
[27.475744, 26.636385, 30.307617, 29.956480]]


p6 = [[37.877502, 36.778084, 31.032320, 30.422783],
[37.489697, 37.609760, 30.588127, 30.408993],
[37.831009, 36.161247, 30.847233, 30.758625],
[37.370945, 36.646271, 31.127777, 30.486977],
[37.518497, 36.347332, 30.858335, 30.350784],
[38.172607, 37.048134, 30.931072, 30.468065],
[37.944897, 37.310848, 30.938208, 30.281406],
[38.029247, 36.659103, 30.982080, 30.366720],
[37.516129, 36.734306, 30.711040, 30.165855],
[37.391327, 37.186817, 31.127520, 30.268511]]


p7 = [[49.787037, 47.693634, 31.493568, 30.561888],
[50.114559, 48.184158, 31.425730, 30.627296],
[49.345249, 48.587425, 31.997025, 31.211552],
[49.262814, 48.156673, 31.696127, 30.777218],
[49.864002, 47.703808, 31.136736, 30.754814],
[50.427105, 48.551712, 31.581089, 30.780064],
[49.986244, 48.507809, 31.440386, 31.078337],
[50.269180, 48.366432, 31.779264, 30.798113],
[49.790756, 48.267746, 31.649824, 30.843136],
[49.324677, 48.106884, 31.783745, 30.835041]]


p8 = [[61.709564, 60.414787, 32.375523, 31.399233],
[62.124512, 60.626019, 32.330017, 31.533442],
[62.026787, 60.677120, 32.590305, 31.459490],
[61.804031, 60.349346, 32.605343, 31.447041],
[61.951645, 60.122787, 32.148445, 31.310623],
[62.010300, 60.167652, 32.365406, 31.253313],
[62.162239, 60.213627, 32.354240, 31.205248],
[61.901474, 60.297665, 32.267902, 31.056959],
[62.363644, 60.267391, 32.134209, 31.109503],
[61.973728, 60.039967, 32.638145, 31.591009]]


# averages:
pp1 = np.sum(np.array(p1), axis = 0) / len(p1)
pp2 = np.sum(np.array(p2), axis = 0) / len(p2)
pp3 = np.sum(np.array(p3), axis = 0) / len(p3) 
pp4 = np.sum(np.array(p4), axis = 0) / len(p4)
pp5 = np.sum(np.array(p5), axis = 0) / len(p5)
pp6 = np.sum(np.array(p6), axis = 0) / len(p6)
pp7 = np.sum(np.array(p7), axis = 0) / len(p7)
pp8 = np.sum(np.array(p8), axis = 0) / len(p8)

print pp1
print pp2
print pp3
print pp4
print pp5
print pp6
print pp7
print pp8





# Data to plot
labels = ['vertexTandA', 'scanline', 'aa', 'render']
colors = ['#005511', '#002244', '#004477', '#118899']
explode = (0.1, 0.5, 0.2, 0.1)


P = [pp1, pp2, pp3, pp4, pp5, pp6, pp7, pp8]
titles = ['optimized\nfunctions time percentage', 
          'aabb only\nfunctions time percentage', 
          'backface cull\nfunctions time percentage', 
          'no opt\nfunctions time percentage',
          'MSAA 4x opt\nfunctions time percentage',
          'MSAA4x no opt\nfunctions time percentage',
          'SSAA opt\nfunctions time percentage',
          'SSAA no opt\nfunctions time percentage']


for i in range(8):
    sizes = P[i]
    
    percent = 100.*P[i]/P[i].sum()

    print percent

    # Plot
    fig = plt.figure(facecolor='black')
    #fig.patch.set_alpha(1.0)
    fig.patch.set_facecolor('black')
    plt.rcParams['text.color'] = 'gray'
    plt.rcParams['axes.facecolor'] = 'black'
    plt.rcParams['lines.linewidth'] = 4

    ax = plt.subplot(111, axisbg='black')
    
    pie = ax.pie(sizes, explode=explode, colors=colors, textprops = {'color':'#aaaaaa', 'fontweight':'bold'},
            autopct='%1.1f%%', shadow=True, startangle=140)



    ax.set_ylabel('Test Benchmark Averages', color = '#338888', fontweight='bold')
    ax.set_title(titles[i])
    ax.patch.set_facecolor('black') 
    ax.axis('equal')

    percentlabels = list(labels)
    for j in range(len(percentlabels)):
        percentlabels[j] += (' %.1f' % percent[j])+'''%'''

    plt.legend(pie[0], percentlabels, loc='upper left')

    plt.savefig('piechart_%d.png' % i, bbox_inches='tight', facecolor='black')
    #plt.show()
    




fig = plt.figure(facecolor='black')
#fig.patch.set_alpha(1.0)
fig.patch.set_facecolor('black')
plt.rcParams['text.color'] = 'gray'
plt.rcParams['axes.facecolor'] = 'black'
plt.rcParams['lines.linewidth'] = 4

N = 8
t1 = [pp1[0], pp2[0], pp3[0], pp4[0], pp5[0], pp6[0], pp7[0], pp8[0]]
t2 = [pp1[1], pp2[1], pp3[1], pp4[1], pp5[1], pp6[1], pp7[1], pp8[1]]
t3 = [pp1[2], pp2[2], pp3[2], pp4[2], pp5[2], pp6[2], pp7[2], pp8[2]]
t4 = [pp1[3], pp2[3], pp3[3], pp4[3], pp5[3], pp6[3], pp7[3], pp8[3]]

ind = np.arange(N)    # the x locations for the groups
width = 0.8       # the width of the bars: can also be len(x) sequence

pl1 = plt.bar(ind, t1, width, color='#004466')
pl2 = plt.bar(ind, t2, width, color='#006699', bottom=t1 )
pl3 = plt.bar(ind, t3, width, color='#11aa88', bottom=t2 )
pl4 = plt.bar(ind, t4, width, color='#ff5555', bottom=t3 )

plt.ylabel('Time in ms', color = '#338888', fontweight='bold')
plt.title('Optimization and post processing benchmark')
plt.xticks(ind + width/2., ('optimized', 
                            'aabb\nonly', 
                            'backface\ncull', 
                            'no opt',
                            'MSAA 4x\nopt',
                            'MSAA4x\nno opt',
                            'SSAA\nopt',
                            'SSAA\nno opt'),
            color = '#333333',
            fontweight='bold')

plt.yticks(np.arange(0, 1000, 100), color = '#333333', fontweight='bold')
plt.legend([pl1[0], pl2[0], pl3[0], pl4[0]], labels, loc=0)

plt.savefig('boxchart.png', bbox_inches='tight', facecolor='black')
#plt.show()





# no scanline





