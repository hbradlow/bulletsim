from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Circle, PathPatch
import pylab
import numpy as np
import scipy
from scipy import optimize
import math
class Arrow3D(FancyArrowPatch):
    """
        Code from http://stackoverflow.com/questions/11140163/python-matplotlib-plotting-a-3d-cube-a-sphere-and-a-vector
    """
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

class Axis3D:
    def __init__(self,transform,*args,**kwargs):
        self.transform = np.array(transform)
        origin = self.transform.take([3],1).flatten().tolist()

        xaxis = np.dot(self.transform,np.array([1,0,0,0]))
        yaxis = np.dot(self.transform,np.array([0,1,0,0]))
        zaxis = np.dot(self.transform,np.array([0,0,1,0]))

        axis = [xaxis,yaxis,zaxis]
        axis = [a/(np.linalg.norm(a)*8.) for a in axis]
        axis = [a.tolist() for a in axis]
        
        self.arrows = []

        colors = ['r','g','b']
        for a,color in zip(axis,colors):
            self.arrows.append(Arrow3D([origin[0],origin[0]+a[0]],[origin[1],origin[1]+a[1]],[origin[2],origin[2]+a[2]],mutation_scale=20,lw=3,arrowstyle="-|>", color=color))

class Transform:
    def __init__(self):
        self.is_valid = False
        self.transform = [[0 for i in range(4)] for j in range(4)]
        
class TransformProcessor:
    def __init__(self,filename="transforms.txt"):
        self.filename = filename
        self.transforms = []
        self.translations = []
        self.rotations = []

    def load_file(self):
        f = open(self.filename)
        line = f.readline()
        while line:
            if "TRANSFORM" in line:
                t = Transform()
                t.is_valid = f.readline().strip() == "1"
                for i in range(4):
                    line = f.readline()
                    t.transform[i] = [float(a) for a in line.strip().split(" ")]
                self.transforms.append(t)
            line = f.readline()
        self.update_transform_components()

    def update_transform_components(self):
        self.translations = list(self.calculate_translations())
        self.rotations = list(self.calculate_rotations())

    def process(self):
        self.calculate_plane_to_yz_transform()
        self.transform_plane_to_yz()
        self.calculate_best_fit_circle()
        #self.transform_plane_to_yz(inverse=True) #not sure why this isnt working...

    def calculate_best_fit_circle(self):
        """
            http://math.stackexchange.com/questions/214661/circle-least-squares-fit
        """
        A = []
        b = []
        for point in self.translations:
            A.append([point[1]*2,point[2]*2,1])
            b.append(point[1]**2 + point[2]**2)
        x = scipy.optimize.nnls(np.array(A),np.array(b))
        self.circle = x[0].tolist()
        self.circle[2] = math.sqrt(self.circle[2] + self.circle[0]**2 + self.circle[1]**2)

    def transform_plane_to_yz(self,inverse=False):
        for translation,rotation,(transform_index,transform) in zip(self.project_transforms_to_plane(),self.rotations,enumerate(self.transforms)):
            if inverse:
                t = np.linalg.inv(np.array(self.plane_to_yz_transform))
            else:
                t = np.array(self.plane_to_yz_transform)
            translation = np.dot(t,np.array(translation+[1])).tolist()[0:3]
            transform.transform = [rotation[index] + [translation[index]] for index in range(3)]
            transform.transform += [0,0,0,1]
            self.transforms[transform_index] = transform
        self.update_transform_components()

    def calculate_translations(self):
        for transform in self.transforms:
            yield [t[3] for t in transform.transform[0:3]]

    def calculate_rotations(self):
        for transform in self.transforms:
            yield [t[0:3] for t in transform.transform[0:3]]

    def best_fit_plane(self):
        def zeros(i):
            return [0 for a in range(i)]
        A = np.array([zeros(3) for j in range(3)])
        b = np.array(zeros(3))
        for point in self.translations:
            A = np.add(np.array([   [point[0]*point[0], point[0]*point[1],  point[0]],
                        [point[0]*point[1], point[1]*point[1],  point[1]],
                        [point[0],          point[1],           1]]),A)
            b = np.add(np.array([point[0]*point[2],point[1]*point[2],point[2]]),b)
        x = np.linalg.solve(A,b)
        return x

    def calculate_plane_to_yz_transform(self):
        x = self.best_fit_plane()
        normal = [x[0],x[1],-1]
        theta = math.atan(-normal[2]/normal[1])
        alpha = math.atan(-(math.cos(theta)*normal[1] + math.sin(theta)*normal[2])/normal[0])
        transform = [   [math.cos(alpha),-math.sin(alpha)*math.cos(theta),math.sin(alpha)*math.sin(theta),0],
                        [math.sin(alpha),math.cos(alpha)*math.cos(theta),-math.cos(alpha)*math.sin(theta),0],
                        [0,math.sin(theta),math.cos(theta),0],
                        [0,0,0,1]]
        self.plane_to_yz_transform = transform

    def project_transforms_to_plane(self):
        x = self.best_fit_plane()
        origin = np.array([0,0,x[2]])
        normal = np.array([x[0],x[1],-1])
        def project_point_to_plane(point):
            to_p = np.add(point,np.dot(-1,origin))
            return np.add(point,np.dot(-1,np.dot(normal,np.dot(normal,to_p))))
        return [project_point_to_plane(point).tolist() for point in self.translations]

    def show(self,label="Translations"):
        fig = plt.figure(label)
        ax = fig.add_subplot(111,projection="3d",aspect=1)
        x = [t[0] for t in self.translations]
        y = [t[1] for t in self.translations]
        z = [t[2] for t in self.translations]
        def ave(l):
            return reduce(lambda x,y: x+y,l)/len(l)
        """
        for t in self.transforms:
            a = Axis3D(t.transform)
            for arrow in a.arrows:
                ax.add_artist(arrow)
        """
        ax.scatter(x,y,z)
        circle = Circle((self.circle[0], self.circle[1]), self.circle[2],fill=False)
        ax.add_patch(circle)
        art3d.pathpatch_2d_to_3d(circle, z=0, zdir="x")
        ax.auto_scale_xyz([-.5, .5], [-.5, .5], [-0, 1])
        plt.show()
