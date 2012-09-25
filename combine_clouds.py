import os
import re

def sort_key(file):
    number = re.match(r'.*?(\d+)\.pcd',file).group(1)
    return int(number)
base_dir = "pcd_files"
files = sorted(os.listdir(base_dir),key=sort_key)
args = ""
for f in files[0::15]:
    args += " " + base_dir + "/" + f
#run the algorithm
os.system("./bin/test_cloud_reconstruction " + args)

#downsample the output
os.system("pcl_voxel_grid -leaf 0.003,0.003,0.003 test.pcd test2.pcd")
