import os
import subprocess
import re
import sh

#compile the command from the directory of clouds
def sort_key(file):
    number = re.match(r'.*?(\d+)\.pcd',file).group(1)
    return int(number)
base_dir = "pcd_files2"
files = sorted(os.listdir(base_dir),key=sort_key)
args = ""
for f in files[0::100]:
    args += base_dir + "/" + f + " "
command = "./bin/test_cloud_reconstruction " + args
if len(command)>600:
    print command[0:600] + "......(more args)"
else:
    print command

#run the command
proc = subprocess.Popen(command.strip().split(" "),stdout=subprocess.PIPE)
translations = []
poses = []
xs = []
ys = []
zs = []
while True:
    line = proc.stdout.readline()
    if line == "":
        break;
    elif "Transform:" in line:
        #extract the list of poses from the output
        match = re.match("^Transform:\s*(.+?)\s+(.+?)\s+(.+?)\s*$",line)
        translations.append((match.group(1),match.group(2),match.group(3)))
    elif "X:" in line:
        #extract the list of poses from the output
        match = re.match("^X:\s*(.+?)\s+(.+?)\s+(.+?)\s*$",line)
        xs.append((match.group(1),match.group(2),match.group(3)))
    elif "Y:" in line:
        #extract the list of poses from the output
        match = re.match("^Y:\s*(.+?)\s+(.+?)\s+(.+?)\s*$",line)
        ys.append((match.group(1),match.group(2),match.group(3)))
    elif "Z:" in line:
        #extract the list of poses from the output
        match = re.match("^Z:\s*(.+?)\s+(.+?)\s+(.+?)\s*$",line)
        zs.append((match.group(1),match.group(2),match.group(3)))
    else:
        print line
        
def plot(points,label):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure(label)
    ax = fig.add_subplot(111,projection="3d")
    x = [float(a[0]) for a in points]
    y = [float(a[1]) for a in points]
    z = [float(a[2]) for a in points]
    def ave(l):
        return reduce(lambda x,y: x+y,l)/len(l)
    ax.scatter(x,y,z)
    plt.show()

debug = False
if debug:
    plot(translations,"Translations")
    plot(xs,"Affect on X axis")
    plot(ys,"Affect on Y axis")
    plot(zs,"Affect on Z axis")

import convert
subprocess.Popen("../PoissonRecon-amd64 --in test.pts --out out --depth 6".split(" ")).wait()
subprocess.Popen("meshlab out.ply".split(" "))
