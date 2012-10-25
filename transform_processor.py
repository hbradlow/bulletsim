from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.patches
import pylab
import numpy as np
import scipy
from scipy import optimize
import math
class Arrow3D(FancyArrowPatch):
    """
        An arrow object for a matplotlib 3d plot.
        
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
    """
        Object to store an axis triple.
    """
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
            a = np.dot(2,np.array(a)).tolist() # make the vector longer
            self.arrows.append(Arrow3D([origin[0],origin[0]+a[0]],[origin[1],origin[1]+a[1]],[origin[2],origin[2]+a[2]],mutation_scale=20,lw=1,arrowstyle="-|>", color=color))

class Circle:
    def __init__(self,origin,radius):
        self.origin = origin
        self.radius = radius
        self.axis = None
    def __repr__(self):
        return "center: " + str(self.origin) + ", radius: " + str(self.radius)
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
        self.show_transform_axis = True
        self.angle_per_index = None
    def update_angle_per_index(self,angle):
        alpha = .2
        print "angle",angle
        if self.angle_per_index:
            self.angle_per_index = (1-alpha)*self.angle_per_index + alpha*angle
        else:
            self.angle_per_index = angle
    def output_to_file(self):
        """
            Write the transforms to a text file.
        """
        f = open(self.filename,"w")
        for transform in self.transforms:
            f.write("TRANSFORM\n")
            f.write("1\n")
            #f.write(("1" if transform.is_valid else "0") + "\n")
            for row in transform.transform:
                f.write(" ".join([str(num) for num in row]) + "\n")
            f.write("\n")
    def load_from_file(self):
        """
            Load the transform from a text file.
        """
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
        """
            Update the cached components of the transform.
        """
        self.calculate_translations()
        self.calculate_rotations()

    def ave_height(self):
        num = 0
        total = 0
        for (index,translation) in enumerate(self.translations):
            if self.transforms[index].is_valid:
                num+=1
                total += translation[2]
        return float(total)/num
    def process(self):
        """
            Perform some transformations on the transform matricies.
        """
        self.calculate_plane_to_yz_transform()
        self.transform_transforms(self.plane_to_yz_transform)
        self.calculate_flatten_transform()
        self.transform_transforms(self.flatten_transform)

        self.calculate_best_fit_circle()
        self.mark_transforms_invalid()
        #self.sort_transforms_by_circle_angle()
        self.interpolate_transforms_along_circle()

        #undo the intermediate transformations
        self.transform_transforms(self.flatten_transform,inverse=True)
        self.transform_transforms(self.plane_to_yz_transform,inverse=True)


    def mark_transforms_invalid(self,error_margin=.1):
        """
            Mark transforms that do not fit the circle well as invalid.
        """
        for (index,translation) in enumerate(self.translations):
            origin = np.array(self.circle.origin)
            p = np.array(translation)
            v = np.add(p,np.dot(-1,origin))
            change = np.linalg.norm(v)/self.circle.radius
            if change > 1+error_margin or change < 1-error_margin: 
                self.transforms[index].is_valid = False

    def sort_transforms_by_circle_angle(self):
        def map_index(l,indexes):
            new_l = [l[i] for i in indexes]
            return new_l
        base_vector = np.array([0,0,1])
        angles = []
        for translation in self.translations:
            vector = np.add(np.array(translation),np.dot(-1,np.array(self.circle.origin)))
            div = np.dot(vector,base_vector)/(np.linalg.norm(vector)*np.linalg.norm(base_vector))
            angle = math.acos(div)
            angles.append(angle)
        print angles
        indexes = [i[0] for i in sorted(enumerate(self.translations),key=lambda x: angles[x[0]])]
        print indexes
        self.translations = map_index(self.translations,indexes)
        self.rotations = map_index(self.rotations,indexes)
        self.transforms = map_index(self.transforms,indexes)

    def valid_enclosing_transforms(self,index):
        length = len(self.transforms)
        enclosing_transforms = [None,None]
        start = index
        end = index
        while not (enclosing_transforms[0] is not None and enclosing_transforms[1] is not None):
            if self.transforms[start].is_valid:
                enclosing_transforms[0] = start
            else:
                start = (start-1)%length
            if self.transforms[end].is_valid:
                enclosing_transforms[1] = end
            else:
                end = (end+1)%length

        return enclosing_transforms
    def slerp_rotations(self,index_start,index_current,index_end):
        """
            http://en.wikipedia.org/wiki/Slerp
            http://cgkit.sourceforge.net/doc2/cgtypes.html?highlight=slerp#cgkit.cgtypes.slerp
        """
        from cgkit.cgtypes import quat, mat3, slerp
        def get_quaternion_from_matrix(matrix):
            q = quat()
            mat = mat3([num for row in matrix for num in row])
            mat = mat.decompose()[0]
            q.fromMat(mat)
            return q

        if index_start == index_end:
            t = 0
        else:
            t = float(index_current-index_start)/float(index_end-index_start)

        q_start = get_quaternion_from_matrix(self.rotations[index_start])
        q_end = get_quaternion_from_matrix(self.rotations[index_end])

        if t==0:
            q_current = q_start
        else:
            q_current = slerp(t,q_start,q_end)
        mat_current = q_current.toMat3()

        list_current = []
        for i in range(3):
            list_current.append([])
            for j in range(3):
                list_current[i].append(mat_current[i,j])
        return list_current

    def slerp_translations(self,index_start,index_current,index_end):
        """
            http://en.wikipedia.org/wiki/Slerp
        """
        def set_norm(vector,length):
            return np.dot(vector,length/np.linalg.norm(vector))

        origin = np.array(self.circle.origin)

        p_start = np.array(self.translations[index_start])
        p_end = np.array(self.translations[index_end])

        v_start = np.add(p_start,np.dot(-1,origin))
        v_end = np.add(p_end,np.dot(-1,origin))
        v_start = set_norm(v_start,self.circle.radius)
        v_end = set_norm(v_end,self.circle.radius)

        div = np.dot(v_start,v_end)/(np.linalg.norm(v_start)*np.linalg.norm(v_end))
        if div<=1:
            angle = math.acos(div)
        else:
            angle = 0.

        if index_start == index_end:
            t = 0
        else:
            t = float(index_current-index_start)/float(index_end-index_start)

        if angle == 0:
            v_current = v_start
        else:
            v_current = np.dot((math.sin((1-t)*angle)/math.sin(angle)),v_start) + np.dot((math.sin(t*angle)/math.sin(angle)),v_end)
        p_current = np.add(v_current,origin)

        return p_current.tolist()[0:3]

    def angle_between_translations(self,t1,t2):
        base_vector = np.array([0,0,1])
        def circle_angle_of_translation(t):
            vector = np.add(np.array(t),np.dot(-1,np.array(self.circle.origin)))
            div = np.dot(vector,base_vector)/(np.linalg.norm(vector)*np.linalg.norm(base_vector))
            return math.acos(div)
        return circle_angle_of_translation(t2)-circle_angle_of_translation(t1)

    def interpolate_transforms_along_circle(self):
        good_index = None
        for (index,transform) in enumerate(self.transforms):
            enclosing_transforms = self.valid_enclosing_transforms(index)
            self.translations[index] = self.slerp_translations(enclosing_transforms[0],index,enclosing_transforms[1])
            self.rotations[index] = self.slerp_rotations(enclosing_transforms[0],index,enclosing_transforms[1])
            """
            else:
                if good_index:
                    self.update_angle_per_index(self.angle_between_translations(self.translations[good_index],self.translations[index])/float(index-good_index))
                print self.angle_per_index
            """

        self.calculate_transforms()
                
    def calculate_best_fit_circle(self):
        """
            http://math.stackexchange.com/questions/214661/circle-least-squares-fit
        """
        A = []
        b = []
        for point,transform in zip(self.translations,self.transforms):
            if transform.is_valid:
                A.append([point[1]*2,point[2]*2,1])
                b.append(point[1]**2 + point[2]**2)
        x = np.linalg.lstsq(np.array(A),np.array(b))
        self.circle = Circle([0] + x[0].tolist()[0:2],x[0].tolist()[2])
        self.circle.radius = math.sqrt(self.circle.radius + self.circle.origin[1]**2 + self.circle.origin[2]**2)
        self.circle.axis = [1,0,0]

    def transform_transforms(self,transform_matrix,inverse=False):
        """
            Transform the translations using a transform matrix
        """
        if inverse:
            t = np.linalg.inv(np.array(transform_matrix))
        else:
            t = np.array(transform_matrix)
        """
        for (index,translation) in enumerate(self.translations):
            translation = np.dot(t,np.array(translation+[1])).tolist()[0:3]
            self.translations[index] = translation
        """
        for (index,transform) in enumerate(self.transforms):
            transform = np.dot(t,np.array(transform.transform)).tolist()
            self.transforms[index].transform = transform
        self.update_transform_components()

    def calculate_transforms(self):
        """
            Calculate the transforms from the translations and rotations
        """
        new_transforms = []
        for translation,rotation,transform in zip(self.translations,self.rotations,self.transforms):
            new_transform = Transform()
            new_transform.is_valid = transform.is_valid
            new_transform.transform = [rotation[index] + [translation[index]] for index in range(3)] + [[0,0,0,1]]
            new_transforms.append(new_transform)
        self.transforms = new_transforms
    
    def calculate_translations(self):
        """
            Calculate the translations from the transforms
        """
        new_translations = []
        for transform in self.transforms:
            new_translations.append([t[3] for t in transform.transform[0:3]])
        self.translations = new_translations

    def calculate_rotations(self):
        """
            Calculate the rotations from the transforms
        """
        new_rotations = []
        for transform in self.transforms:
            new_rotations.append([t[0:3] for t in transform.transform[0:3]])
        self.rotations = new_rotations

    def best_fit_plane(self):
        """
            Find the plane that best fits the set of translations
        """
        def zeros(i):
            return [0 for a in range(i)]
        A = np.array([zeros(3) for j in range(3)])
        b = np.array(zeros(3))
        for point,transform in zip(self.translations,self.transforms):
            if transform.is_valid:
                A = np.add(np.array([   [point[0]*point[0], point[0]*point[1],  point[0]],
                            [point[0]*point[1], point[1]*point[1],  point[1]],
                            [point[0],          point[1],           1]]),A)
                b = np.add(np.array([point[0]*point[2],point[1]*point[2],point[2]]),b)
        x = np.linalg.solve(A,b)
        return x

    def calculate_flatten_transform(self):
        """
            Calculate the transform to move all the translation points into the yz plane. Basically just remove the x values.
        """
        def ave(l):
            return reduce(lambda x,y: x+y,l)/len(l)
        a = ave([t[0] for t,transform in zip(self.translations,self.transforms) if transform.is_valid])
        transform = [   [1,0,0,-a],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]]
        self.flatten_transform = transform

    def calculate_plane_to_yz_transform(self):
        """
            Calculate the transform to rotate the plane of the circle into the yz plane.
        """
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
        """
            Project the transforms onto the best fit plane.
        """
        x = self.best_fit_plane()
        origin = np.array([0,0,x[2]])
        normal = np.array([x[0],x[1],-1])
        def project_point_to_plane(point):
            to_p = np.add(point,np.dot(-1,origin))
            return np.add(point,np.dot(-1,np.dot(normal,np.dot(normal,to_p))))
        return [project_point_to_plane(point).tolist() for point in self.translations]

    def show(self,label="Translations"):
        """
            Display the transforms on a 3d plot.
        """
        fig = plt.figure(label)
        ax = fig.add_subplot(111,projection="3d",aspect=1)
        x = []; y = []; z = []
        x_n = []; y_n = []; z_n = []
        for translation,transform in zip(self.translations,self.transforms):
            if transform.is_valid:
                x.append(translation[0])
                y.append(translation[1])
                z.append(translation[2])
            if not transform.is_valid:
                x_n.append(translation[0])
                y_n.append(translation[1])
                z_n.append(translation[2])
        ax.scatter(x,y,z,color="b",s=200)
        ax.scatter(x_n,y_n,z_n,color="r",s=200)

        if self.show_transform_axis:
            for t in self.transforms:
                a = Axis3D(t.transform)
                for arrow in a.arrows:
                    ax.add_artist(arrow)

        circle_axis = Arrow3D((0,self.circle.axis[0]),(self.circle.origin[1],self.circle.origin[1]+self.circle.axis[1]),(self.circle.origin[2],self.circle.origin[2]+self.circle.axis[2]),mutation_scale=20,lw=3,arrowstyle="-|>", color="g")
        ax.add_artist(circle_axis)

        circle = matplotlib.patches.Circle((self.circle.origin[1], self.circle.origin[2]), self.circle.radius,fill=False)
        ax.add_patch(circle)

        art3d.pathpatch_2d_to_3d(circle, z=0, zdir="x")
        ax.auto_scale_xyz([-.5, .5], [-.5, .5], [-0, 1])
        plt.show()
