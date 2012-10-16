#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>

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
typedef pcl::PointCloud<pcl::PointNormal> HCloud;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr HCloudPtr;

//these values are measured values from the setup
//specifies the bouding box around the object being scanned
#define X_LOWER_BOUND           -0.33
#define X_UPPER_BOUND           -0.18
#define Y_LOWER_BOUND           -0.1
#define Y_UPPER_BOUND           0.1
#define VERTICAL_LOWER_BOUND    0.01

//size of leaf to use for downsampling
#define LEAF_SIZE               0.002f

//flags
#define CALCULATE_NORMALS

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
int loadCloud(char * filename,pcl::PointCloud<pcl::PointNormal>::Ptr cloud,ColorCloudPtr color_cloud){
    //load a pcd and convert it into a ColorCloud and a PointNormal cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile(filename, *tmp) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return -1;
    }
    //do some initial filtering by rejecting points too far from the camera
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.2);
    pass.setKeepOrganized(true);
    pass.filter (*tmp_filtered);
    *tmp = *tmp_filtered;

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    //calculate normals for the cloud - a very slow part of the process
    
        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        ne.setInputCloud (tmp);
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.01);
        ne.compute (*normals);

    cloud->clear();
    color_cloud->clear();
    for(int i = 0; i<tmp->points.size(); i++){
        pcl::PointXYZRGBA p1 = tmp->points[i];
        pcl::Normal n1 = normals->points[i];
        n1.normal_x = 0;
        n1.normal_y = 0;
        n1.normal_z = 1;
        pcl::PointNormal p2;
        ColorPoint p3;
        p2.x = p1.x;
        p2.y = p1.y;
        p2.z = p1.z;
        p3.x = p1.x;
        p3.y = p1.y;
        p3.z = p1.z;
        p2.normal_x = n1.normal_x;
        p2.normal_y = n1.normal_y;
        p2.normal_z = n1.normal_z;
        p3.r = p1.r;
        p3.g = p1.g;
        p3.b = p1.b;
        cloud->push_back(p2);
        color_cloud->push_back(p3);
    }
    cloud->width = tmp->width;
    cloud->height = tmp->height;
    color_cloud->width = tmp->width;
    color_cloud->height = tmp->height;
    return 1;
}
void filterByLocation(HCloudPtr input,HCloudPtr output){
    //strip out points not in the correct region
    HCloudPtr tmp (new HCloud);
    *tmp = *input;

    pcl::PassThrough<pcl::PointNormal> pass;
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (VERTICAL_LOWER_BOUND, 1.0);
    pass.filter (*output);
    *tmp = *output;

    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (Y_LOWER_BOUND, Y_UPPER_BOUND);
    pass.filter (*output);
    *tmp = *output;

    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (X_LOWER_BOUND, X_UPPER_BOUND);
    pass.filter (*output);
    *tmp = *output;
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

    float theta = atan(-normal(2)/normal(1));
    float alpha = atan(-(cos(theta)*normal(1) + sin(theta)*normal(2))/normal(0));
    Eigen::Matrix4f transform;
    transform <<    cos(alpha),-sin(alpha)*cos(theta),sin(alpha)*sin(theta),    -origin(0),
                    sin(alpha),cos(alpha)*cos(theta),-cos(alpha)*sin(theta),    -origin(1),
                    0,sin(theta),cos(theta),                                    -origin(2),
                    0,0,0,1;
    
    Eigen::Vector3f p3_tmp = p-(normal*normal.dot(to_p));
    Eigen::Vector4f p4_tmp;
    p4_tmp << p3_tmp(0),p3_tmp(1),p3_tmp(2),1;
    p4_tmp = transform * p4_tmp;
    return Eigen::Vector3f(p4_tmp(0),p4_tmp(1),p4_tmp(2));
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
    pass.setFilterLimits (VERTICAL_LOWER_BOUND, VERTICAL_LOWER_BOUND+.05);
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
        for(float y = min; y<=max; y+=.001){
            pcl::PointNormal p;
            p.x = x;
            p.y = y;
            p.z = VERTICAL_LOWER_BOUND;
            master->push_back(p);
        }
    }
}

int main(int argc, char*argv[]){
    ColorCloudPtr tmp (new ColorCloud); //a buffer to store the cloud that is loaded from disk
    pcl::PointCloud<pcl::PointNormal>::Ptr current (new pcl::PointCloud<pcl::PointNormal>); //buffer to hold the cloud for the current frame

    pcl::PointCloud<pcl::PointNormal>::Ptr master (new pcl::PointCloud<pcl::PointNormal>); //buffer to hold the concatenated point cloud of all frames
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered (new pcl::PointCloud<pcl::PointNormal>);//buffer to place filtered intermediate clouds


    //keep track of all the frames
    std::vector<Eigen::Matrix4f> transforms; //the transform of the checkerboard for the frame
    std::vector< pcl::PointCloud<pcl::PointNormal> > clouds; //the cloud for a frame
    std::vector< pcl::PointCloud<pcl::PointNormal> > clouds1; //the cloud for a frame
    std::vector<int> success_mask; //indicates if a checkerboard was located in the frame

    if(argc>1){
        // Reorient each cloud to line the checkerboards up and concatenate them all into a "master" cloud

        //collect some stats to tell the user how successful the checkerboard location was
        int found_boards = 0;
        int total_clouds = 0;
        vector<char *> fails; //keep track of the files that failed, in case the user knows something about those files

        cout << "Reorienting the clouds to match their checkerboards..." << endl;
        //load all of the clouds into the "clouds" vector
        for(int i=1; i<argc; i++)
        {
            //load the cloud from the file
            if(loadCloud(argv[i],current,tmp) == -1)
                return -1;
            
            clouds.push_back(*current);

            total_clouds ++;
            //update the user of the progress
            if(total_clouds%3==0)
                cout << (float)total_clouds/argc * 100 << " percent complete..." << endl;

            //calculate the transform of the board
            Eigen::Matrix4f transform;
            int found = getChessBoardPose(tmp,6,9,.0172,transform);
            transforms.push_back(transform);
            if(found){
                success_mask.push_back(1); //indicate that the checkerboard was found
                found_boards ++;
            }
            else
            {
                success_mask.push_back(0); //indicate that the checkerboard was not found
                fails.push_back(argv[i]);
            }
        }

        Eigen::Vector3f x = fitPlaneToTransformTranslations(transforms,success_mask); //get the parameters of the best fit plane to the checkerboard translations
        for(int i  = 0; i< transforms.size(); i++){
            //transforms[i] = projectTransformToPlane(transforms[i],x);
        }
        projectTransformTranslationsToPlane(x,&transforms,success_mask); //project the translations onto the plane

        pcl::PointCloud<pcl::PointNormal>::Ptr master1 (new pcl::PointCloud<pcl::PointNormal>);
        for(int i = 0; i<clouds.size(); i++)
        {
            if(success_mask[i]!=0){
//                Eigen::Matrix4f transform = getSmoothedTransform(transforms,i,0.6,0,success_mask); //exponential smooth of the transforms - 0 means no smoothing
                Eigen::Matrix4f transform = transforms[i]; 
                //print some information about the transform for debugging
                Eigen::Matrix3f rotation;
                rotation <<     transform(0,0),transform(0,1),transform(0,2),
                                transform(1,0),transform(1,1),transform(1,2),
                                transform(2,0),transform(2,1),transform(2,2);
                Eigen::Vector3f x(1,0,0);
                Eigen::Vector3f y(0,1,0);
                Eigen::Vector3f z(0,0,1);
                x = rotation*x;
                y = rotation*y;
                z = rotation*z;
                //cout << "Transform: " << transform(0,3) << " " << transform(1,3) << " " << transform(2,3) << " " << endl;
                cout << "X: " << x(0) << " " << x(1) << " " << x(2) << endl;
                cout << "Y: " << y(0) << " " << y(1) << " " << y(2) << endl;
                cout << "Z: " << z(0) << " " << z(1) << " " << z(2) << endl;




                //transform the cloud and concatenate it with "master"
                pcl::PointCloud<pcl::PointNormal>::Ptr c (new pcl::PointCloud<pcl::PointNormal>(clouds[i]));
                pcl::transformPointCloudWithNormals(*c,*c,transform);
                
                clouds1.push_back(*c);
                for(int j = 0; j<c->points.size(); j++){
                    pcl::PointNormal p1 = c->points[j];
                    master1->push_back(p1);
                }
            }
        }

        //calculate the success percentage
        float percent = (((float)found_boards)/total_clouds)*100.;
        cout << "Found checkerboard in " << found_boards << " of the " << total_clouds << " clouds provided - (" << percent << "%)." << endl;
        //filter the remaining "master" cloud


        //filter by location
        cout << "Filtering the master cloud by relative position to the checkboard..." << endl;

        filterByLocation(master1,master1);

        /*
        for(int i = 0; i< clouds1.size(); i++){
            pcl::PointCloud<pcl::PointNormal>::Ptr c (new pcl::PointCloud<pcl::PointNormal>(clouds1[i]));
            Eigen::Matrix4f transform;
            cout << "HERE" << endl;
            if(getTransformForICPAlignment(master1,c,&transform)){
                pcl::transformPointCloudWithNormals(*c,*c,transform);
                cout << transform << endl;
                pcl::io::savePCDFileASCII ("intermediate3.pcd", *c);
                for(int j = 0; j<c->points.size(); j++){
                    pcl::PointNormal p1 = c->points[j];
                    master->push_back(p1);
                }
            }
        }
        */
        *master = *master1;
        pcl::io::savePCDFileASCII ("intermediate.pcd", *master);


        //filter by statical outliers
        cout << "Filtering the master cloud for statistical outliers..." << endl;
        pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
        sor.setInputCloud (master);
        sor.setMeanK (70);
        sor.setStddevMulThresh (3.0);
        sor.filter (*filtered);
        *master = *filtered;
        
        //downsample the cloud
        cout << "Downsampling the cloud..." << endl;
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*master,*filtered,index);
        pcl::VoxelGrid<pcl::PointNormal> vg;
        vg.setInputCloud (master);
        vg.setLeafSize (LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);
        vg.filter (*filtered);
        *master = *filtered;


        //do a movingleastsquares smooth of the points
        cout << "Reconstructing..." << endl;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal> mls_points;
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
        mls.setInputCloud (master);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.01);
        mls.setOutputNormals(mls_normals);
        mls.reconstruct (mls_points);
        *master = mls_points;

        /*
        //filter by statical outliers
        cout << "Filtering the master cloud for statistical outliers..." << endl;
        sor.setInputCloud (master);
        sor.setMeanK (100);
        sor.setStddevMulThresh (1.0);
        sor.filter (*filtered);
        *master = *filtered;
        */
        
        //close the bottom of the cloud to make a semi inclosed surface
        cout << "Closing the bottom of the shape..." << endl;
//        closeCloud(master);

        //save the output to a file
        pcl::io::savePCDFileASCII ("output.pcd", *master);
    }
    return (0);
}
