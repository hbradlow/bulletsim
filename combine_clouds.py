import os
import subprocess
import re
import sh

def sort_key(file):
    number = re.match(r'.*?(\d+)\.pcd',file).group(1)
    return int(number)
base_dir = "pcd_files"
files = sorted(os.listdir(base_dir),key=sort_key)
args = ""
for f in files[0::10]:
    args += base_dir + "/" + f + " "
command = "./bin/test_cloud_reconstruction " + args
if len(command)>600:
    print command[0:600] + "......(more args)"
else:
    print command
#run the algorithm
#s = os.system(command)
#print sh.sudo(command)
proc = subprocess.Popen(command.strip().split(" "),stdout=subprocess.PIPE)
points = []
while True:
    line = proc.stdout.readline()
    print line
    if line == "":
        break;
    if "Transform" in line:
        match = re.match("^Transform:\s*(.+?)\s+(.+?)\s+(.+?)\s*$",line)
        points.append((match.group(1),match.group(2),match.group(3)))
        
import matplotlib.pyplot as plt
x = [a[0] for a in points]
y = [a[1] for a in points]
plt.plot(x,y)
plt.show()
#downsample the output
#os.system("pcl_voxel_grid -leaf 0.01,0.01,0.01 test.pcd test2.pcd")
