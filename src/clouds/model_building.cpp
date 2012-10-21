#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>
#include <iostream>
#include <fstream>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include "clouds/utils_pcl.h"

#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/transforms.h>
#include "clouds/pcl_typedefs.h"
#include "clouds/get_chessboard_pose.h"
#include "clouds/cloud_ops.h"

using namespace std;
typedef pcl::PointNormal HPoint;
typedef pcl::PointCloud<pcl::PointNormal> HCloud;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr HCloudPtr;

//these values are measured values from the setup
//specifies the bouding box around the object being scanned
#define X_LOWER_BOUND           -0.33
#define X_UPPER_BOUND           -0.18
#define Y_LOWER_BOUND           -0.1
#define Y_UPPER_BOUND           0.1
#define Z_LOWER_BOUND           0.01
#define Z_UPPER_BOUND           1.0

//size of leaf to use for downsampling
#define LEAF_SIZE               0.002f

//flags
#define CALCULATE_NORMALS

//code to split a string by a delimeter
//found here: http://stackoverflow.com/questions/236129/splitting-a-string-in-c
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}
//read a file full of transformations into memory
void readTransforms(string filename, vector<Eigen::Matrix4f>* transforms, vector<int>* success_mask){
    string line;
    ifstream file (filename.c_str());
    if (file.is_open())
    {
        while ( file.good() )
        {
            getline (file,line);
            if(line.compare("TRANSFORM") == 0){
                getline (file,line);
                success_mask->push_back(atoi(line.c_str()));
                Eigen::Matrix4f transform(4,4);
                for(int i = 0; i<4; i++){
                    getline (file,line);
                    vector<string> numbers = split(line,' ');
                    for(int j = 0; j<numbers.size(); j++){
                        transform(i,j) = atof(numbers[j].c_str());
                    }
                }
                transforms->push_back(transform);
            }
        }
        file.close();
    }
}
//write a list of transformations to a file
void writeTransforms(string filename, vector<Eigen::Matrix4f>* transforms, vector<int>* success_mask){
    ofstream file;
    file.open(filename.c_str());
    for(int i = 0; i<transforms->size(); i++){
        file << "TRANSFORM" << endl;
        file << (*success_mask)[i] << endl;
        Eigen::Matrix4f transform = (*transforms)[i];
        for(int r = 0; r<transform.rows(); r++){
            for(int c = 0; c<transform.cols(); c++)
                file << transform(r,c) << " ";
            file << endl;
        }
        file << endl;
    }
    file.close();
}
void getColorCloudFromPointNormalCloud(pcl::PointCloud<pcl::PointNormal>::Ptr a, ColorCloudPtr b){
    b->clear();
    for(int i = 0; i<a->size(); i++){
        ColorPoint c;
        pcl::PointNormal p = a->points[i];
        c.x = p.x;
        c.y = p.y;
        c.z = p.z;
        c.r = 255;
        c.g = 255;
        c.b = 255;
        b->push_back(c);
    }
    b->width = a->width;
    b->height = a->height;
}
void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,string axis, float lower, float upper){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (lower, upper);
    pass.setKeepOrganized(true);
    pass.filter (*tmp);
    *cloud = *tmp;
}
void passThroughFilter(HCloudPtr cloud,string axis, float lower, float upper){
    HCloudPtr tmp (new HCloud);
    pcl::PassThrough<HPoint> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (lower, upper);
    pass.filter (*tmp);
    *cloud = *tmp;
}
int loadCloud(char * filename,pcl::PointCloud<pcl::PointNormal>::Ptr cloud,ColorCloudPtr color_cloud){
    //load a pcd and convert it into a ColorCloud and a PointNormal cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr loaded_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile(filename, *loaded_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return -1;
    }
    //do some initial filtering by rejecting points too far from the camera
    passThroughFilter(loaded_cloud,"z",0,1.2);

    cloud->clear();
    color_cloud->clear();
    for(int i = 0; i<loaded_cloud->points.size(); i++){
        //the current data
        pcl::PointXYZRGBA p1 = loaded_cloud->points[i];

        //format it as a pointnormal
        pcl::PointNormal p2;
        p2.x = p1.x;
        p2.y = p1.y;
        p2.z = p1.z;
        cloud->push_back(p2);

        //format it as a colorpoint
        ColorPoint p3;
        p3.x = p1.x;
        p3.y = p1.y;
        p3.z = p1.z;
        p3.r = p1.r;
        p3.g = p1.g;
        p3.b = p1.b;
        color_cloud->push_back(p3);
    }
    //update the height/width of the clouds to match the loaded cloud
    cloud->width = loaded_cloud->width;
    cloud->height = loaded_cloud->height;
    color_cloud->width = loaded_cloud->width;
    color_cloud->height = loaded_cloud->height;
    return 1;
}
void filterByLocation(HCloudPtr input,HCloudPtr output){
    //strip out points not in the correct region
    *output = *input;
    passThroughFilter(output,"z",Z_LOWER_BOUND,Z_UPPER_BOUND);
    passThroughFilter(output,"y",Y_LOWER_BOUND,Y_UPPER_BOUND);
    passThroughFilter(output,"x",X_LOWER_BOUND,X_UPPER_BOUND);
}
Eigen::Vector3f fitPlaneToTransformTranslations(vector<Eigen::Matrix4f> transforms,vector<int> mask){
    //project all transforms to a plane - the transforms should lie on a circular path, project them to a plane to remove vertical noise
    // Plane fitting algorithm found here: http://stackoverflow.com/questions/1400213/3d-least-squares-plane
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    A <<    0,0,0,
            0,0,0,
            0,0,0;
    b <<    0,0,0;
    for(int i = 0; i<transforms.size(); i++)
    {
        if(mask[i]!=0){
            Eigen::Vector3f p(transforms[i](0,3),transforms[i](1,3),transforms[i](2,3));
            b += Eigen::Vector3f(p(0)*p(2),p(1)*p(2),p(2));
            Eigen::Matrix3f a_tmp;
            a_tmp <<    p(0)*p(0),p(0)*p(1),p(0),
                        p(0)*p(1),p(1)*p(1),p(1),
                        p(0),p(1),1;
            A += a_tmp;
        }
    }
    Eigen::Vector3f x = A.colPivHouseholderQr().solve(b); //plane => x(0)*x + x(1)*y + x(2) = z
    return x;
}
Eigen::Vector3f projectPointToPlane(Eigen::Vector3f p,Eigen::Vector3f x){
    Eigen::Vector3f origin(0,0,x(2));
    Eigen::Vector3f to_p = p-origin;
    Eigen::Vector3f normal(x(0),x(1),-1);

    /*
    float theta = atan(-normal(2)/normal(1));
    float alpha = atan(-(cos(theta)*normal(1) + sin(theta)*normal(2))/normal(0));
    Eigen::Matrix4f transform;
    transform <<    cos(alpha),-sin(alpha)*cos(theta),sin(alpha)*sin(theta),    -origin(0),
                    sin(alpha),cos(alpha)*cos(theta),-cos(alpha)*sin(theta),    -origin(1),
                    0,sin(theta),cos(theta),                                    -origin(2),
                    0,0,0,1;
    
    */
    Eigen::Vector3f p3_tmp = p-(normal*normal.dot(to_p));
    /*
    Eigen::Vector4f p4_tmp;
    p4_tmp << p3_tmp(0),p3_tmp(1),p3_tmp(2),1;
    p4_tmp = transform * p4_tmp;
    return Eigen::Vector3f(p4_tmp(0),p4_tmp(1),p4_tmp(2));
    */
    return p3_tmp;
}
Eigen::Matrix4f projectTransformToPlane(Eigen::Matrix4f t,Eigen::Vector3f x){
    Eigen::Vector3f origin(0,0,x(2));
    Eigen::Vector3f p(t(0,3),t(1,3),t(2,3));
    Eigen::Vector3f to_p = p-origin;
    Eigen::Vector3f normal(x(0),x(1),-1);

    float theta = atan(-normal(2)/normal(1));
    float alpha = atan(-(cos(theta)*normal(1) + sin(theta)*normal(2))/normal(0));
    Eigen::Matrix4f transform;
    transform <<    cos(alpha),-sin(alpha)*cos(theta),sin(alpha)*sin(theta),    -origin(0),
                    sin(alpha),cos(alpha)*cos(theta),-cos(alpha)*sin(theta),    -origin(1),
                    0,sin(theta),cos(theta),                                    -origin(2),
                    0,0,0,1;
    
    Eigen::Vector3f p3_tmp = p-(normal*normal.dot(to_p));
    Eigen::Matrix4f out(t);
    out(0,3) = p3_tmp(0);
    out(1,3) = p3_tmp(1);
    out(2,3) = p3_tmp(2);
    return transform * out;
}
void projectTransformTranslationsToPlane(Eigen::Vector3f x, vector<Eigen::Matrix4f>* transforms, vector<int> mask){
    //project the translation component of the transforms onto a plane defined by the parameters stored in 'x'
    float angle = M_PI*2./transforms->size();
    cout << "STEP: " << angle << endl;
    Eigen::Vector3f previous;
    Eigen::Matrix4f previous_t;
    float previous_i;
    for(int i = 0; i<transforms->size(); i++)
    {
        Eigen::Matrix4f t = (*transforms)[i];
        Eigen::Vector3f p(t(0,3),t(1,3),t(2,3));
        //p = projectPointToPlane(p,x);

        if(mask[i]!=0){
            previous = p;
            previous_t = t;
            previous_i = i;
        }
        else{
            t = previous_t;
            float o = 0;
            float time = 0;
            Eigen::Vector3f p2;
            Eigen::Matrix4f t2;
            for(int j = i+1; i<transforms->size(); j++){
                if(mask[j]!=0)
                {
                    o = (j-previous_i)*angle;
                    time = ((float)(i-previous_i))/(j-previous_i);
                    t2 = (*transforms)[j];
                    p2 << t2(0,3),t2(1,3),t2(2,3);
                    p2 = projectPointToPlane(p2,x);
                    break;
                }
            }
            cout << "ANGLE: " << o << endl;
//            p = previous*(sin((1-time)*o)/sin(o)) + p2*(sin(time*o)/sin(o));
        
            t = previous_t*(1-time) + t2*time;
            p << t(0,3),t(1,3),t(2,3);
        }
        cout << "Transform: " << p(0) << " " << p(1) << " " << p(2) << " " << endl;
        (*transforms)[i] <<    t(0,0),t(0,1),t(0,2),p(0),
                            t(1,0),t(1,1),t(1,2),p(1),
                            t(2,0),t(2,1),t(2,2),p(2),
                            t(3,0),t(3,1),t(3,2),t(3,3);
    }
}
Eigen::Matrix4f getSmoothedTransform(vector<Eigen::Matrix4f> transforms, int index, float alpha, int radius,vector<int> mask){
    //returns a smoothed version of the transform at index
    //exponential moving average
    Eigen::Matrix4f transform;
    transform <<    0,0,0,0,
                    0,0,0,0,
                    0,0,0,0,
                    0,0,0,0;
    int size = transforms.size();
    float div = 0.;
    for(int j = -radius; j<=radius; j++){
        int transform_index = ((index+j)%size);
        if(mask[transform_index]!=0){
            float factor = pow(alpha,abs(j));
            transform += factor*transforms[transform_index];
            div += factor;
        }
    }
    if(div!=0){
        transform /= div;
    }
    return transform;
}
int getTransformForICPAlignment(pcl::PointCloud<pcl::PointNormal>::Ptr master, pcl::PointCloud<pcl::PointNormal>::Ptr current, Eigen::Matrix4f* transform){
    //temp clouds
    pcl::PointCloud<pcl::PointNormal>::Ptr master_filtered (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr current_filtered (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr master_filtered_nonan (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr current_filtered_nonan (new pcl::PointCloud<pcl::PointNormal>);
    vector<int> index; //unused, just to store output from removeNaNFromPointCloud

    filterByLocation(current,current);

    *master_filtered_nonan = *master;
    pcl::removeNaNFromPointCloud(*current,index);

    pcl::VoxelGrid<pcl::PointNormal> vg;
    vg.setInputCloud (master_filtered_nonan);
    vg.setLeafSize (LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);
    vg.filter (*master_filtered);

    vg.setInputCloud (current);
    vg.setLeafSize (LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);
    vg.filter (*current_filtered);

    pcl::removeNaNFromPointCloud(*current_filtered,index);
    pcl::removeNaNFromPointCloud(*master_filtered,index);

    pcl::io::savePCDFileASCII ("intermediate2.pcd", *master_filtered);

    if(current_filtered->size()>0 && master_filtered->size()>0){
        pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp; 
        typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;
        boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
        icp.setTransformationEstimation(point_to_plane);
        icp.setInputCloud(current_filtered);
        icp.setInputTarget(master_filtered);
        pcl::PointCloud<pcl::PointNormal> Final;
        icp.align(Final);
        if(icp.hasConverged()){
            cout <<"Converged" << endl;
            *transform = icp.getFinalTransformation();
            return 1;
        }
    }
    return 0;
}
void closeCloud(pcl::PointCloud<pcl::PointNormal>::Ptr master){
    //closes the cloud by adding a plate at the bottom
    pcl::PointCloud<pcl::PointNormal>::Ptr plate (new pcl::PointCloud<pcl::PointNormal>);

    //get the ring of points at the bottom of the cloud
    pcl::PassThrough<pcl::PointNormal> pass;
    pass.setInputCloud (master);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (Z_LOWER_BOUND, Z_LOWER_BOUND+.05);
    pass.filter (*plate);

    //take slices of the ring and add points to fill in the cloud
    for(float x = X_LOWER_BOUND; x<=X_UPPER_BOUND; x+=.001){
        //get just the stip contained by this slice
        pcl::PointCloud<pcl::PointNormal> strip;
        pass.setInputCloud (plate);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x, x+.01);
        pass.filter (strip);
        if(strip.size()==0){
            //if there are no points in this stip, dont do anything
            continue;
        }

        //find the min and max y values of this strip
        float min = 9;
        float max = -9;
        for(int i = 0; i<strip.size(); i++)
        {
            float y = strip.points[i].y;
            if(y<min)
                min = y;
            if(y>max)
                max = y;
        }
        //add points from ymin to ymax for this strip
        for(float y = min+.01; y<=max-.01; y+=.001){
            pcl::PointNormal p;
            p.x = x;
            p.y = y;
            p.z = Z_LOWER_BOUND;
            //point the normal down
            p.normal_x = 0;
            p.normal_y = 0;
            p.normal_z = -1;
            master->push_back(p);
        }
    }
}
