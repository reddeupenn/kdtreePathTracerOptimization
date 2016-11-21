


f = open("farmhouse_bak.obj", 'r')
fout = open("farmhouse.obj", 'w')

scale = 0.2

line = f.readline()
while(line):
    line = f.readline()
    if line.startswith('v '):
        s = line.split()
        line2 = 'v %f %f %f\n' % (float(s[1])*scale, float(s[2])*scale, float(s[3])*scale)
        fout.write(line2)
    else:
        fout.write(line)

f.close()
fout.close()

