"""
    Pipeline:

    test_cloud_reconstruction: takes a list of point clouds and outputs a concatenated cloud of the object

    PoissonRecon-amd64: poisson reconstruction of the point cloud model
"""
import os
import re
import time
from asyncproc import Process
import signal
import sys
import numpy as np
import math

from transform_processor import TransformProcessor

class ReconstructionPipeline:

    def __init__(self,base_dir="pcd_files2",transforms_name="transforms.txt",skip=30,debug=False):
        self.base_dir = base_dir
        self.skip = skip
        self.debug = debug

        self.transforms_name = transforms_name
        self.ply_name = "output.ply"
        self.pcd_name = "output.pcd"
        self.pts_name = "output.pts"

    def collect_data(self):
        """
            Use the pcl app "pcl_openni_recorder" to record the object and store the clouds in a tmp file.
        """
        if self.debug:
            print "Collecting point cloud data"
        try:
            os.mkdir(self.base_dir)
            os.chdir(self.base_dir)
        except OSError:
            os.chdir(self.base_dir)
            os.system("rm *.pcd")
        os.system("kill $(pgrep XnSensorServer)") #this is needed because the process from previous records does not die
        command = "pcl_openni_recorder"
        self.run_command(command,35)
        os.chdir("../")

    def concatenate_clouds(self):
        """
            Call the "concatenate_clouds" executable to combine the clouds into a single object.
        """
        if self.debug:
            print "Cocatenating clouds"
        command = "./bin/concatenate_clouds " + self.get_files_string()
        self.run_command(command)

    def calculate_transforms(self):
        """
            Calculate the poses of the checkerboard for each cloud.
        """
        if self.debug:
            print "Calculating transformation matricies"
        command = "./bin/calculate_checkerboard_transforms " + self.get_files_string()
        self.run_command(command)

    def reconstruct(self):
        """
            Apply a poisson reconstruction to the concatenated cloud.
        """
        if self.debug:
            print "Performing poisson reconstruction"
        import convert
        convert.pcd_to_pts(open(self.pcd_name),open(self.pts_name,"w"))
        command = "../PoissonRecon-amd64 --in " + self.pts_name + " --out " + self.ply_name.split(".")[0] + " --depth 6"
        self.run_command(command)

    def show(self):
        """
            Open the reconstructed mesh in meshlab.
        """
        if self.debug:
            print "Displaying mesh"
        command = "meshlab " + self.ply_name
        self.run_command(command)

    def get_files_string(self):
        """
            Generate a string of all the point clouds that will be processed by parts of the pipeline.
        """
        #compile the command from the directory of clouds
        def sort_key(file):
            number = re.match(r'.*?(\d+)\.pcd',file).group(1)
            return int(number)
        files = sorted(os.listdir(self.base_dir),key=sort_key)
        output = ""
        for f in files[0::self.skip]:
            output += self.base_dir + "/" + f + " "
        return output

    def run_command(self,command,seconds=None):
        """
            Run a command on the system and keep the user updated on the completeness.
        """
        proc = Process(command.strip().split(" "))

        self.display_progress(0)
        start_time = time.time()
        while True:
            time.sleep(.1)
            sys.stdout.flush()
            poll = proc.wait(os.WNOHANG)
            if poll != None:
                break
            line = proc.read()
            if "PERCENT COMPLETE:" in line:
                self.display_progress(float(line.split("PERCENT COMPLETE:")[-1].replace("\n","")))
            if seconds:
                current_time = time.time()
                if current_time-start_time>seconds:
                    break
                self.display_progress(float(current_time-start_time)*100./seconds)
        try:
            proc.kill(signal.SIGTERM)
        except OSError:
            pass
        self.display_progress(100)
        print

    def display_progress(self,percent):
        """
            Display a text based progress bar.
        """
        size = 60
        num_dashes = int(percent*size/100.)
        num_spaces = size-num_dashes
        s = "[" + "".join(["-" for i in range(num_dashes)]) + "".join([" " for i in range(num_spaces)]) + "] " + str(round(percent,2)) + "%"
        print s, "\r",

if __name__ == "__main__":
    pipeline = ReconstructionPipeline(skip=100,debug=True,base_dir="tmp_combine_clouds")
    pipeline.collect_data()
    pipeline.calculate_transforms()
    pipeline.concatenate_clouds()
    pipeline.reconstruct()
    pipeline.show()

def test_processor():
    """
        A test of the transformation processor.
    """
    tp = TransformProcessor()
    tp.load_file()
    tp.process()
    tp.show()
    return tp



# old stuff

"""
translations = []
poses = []
xs = []
ys = []
zs = []
complete = 0.0
while True:
    line = proc.stdout.readline()
    if line == "":
        break;
    elif "PERCENT COMPLETE:" in line:
        complete = float(line.split("PERCENT COMPLETE:")[-1].strip())
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
exit()
        
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
plot(translations,"Translations")
if debug:
    plot(xs,"Affect on X axis")
    plot(ys,"Affect on Y axis")
    plot(zs,"Affect on Z axis")

import convert
subprocess.Popen("../PoissonRecon-amd64 --in test.pts --out out --depth 6".split(" ")).wait()
subprocess.Popen("meshlab out.ply".split(" "))
"""
