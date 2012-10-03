#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>

#include <pcl/registration/icp.h>
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

//these values are measured values from the setup
//specifies the bouding box around the object being scanned
#define X_LOWER_BOUND           -0.33
#define X_UPPER_BOUND           -0.18
#define Y_LOWER_BOUND           -0.1
#define Y_UPPER_BOUND           0.1
#define VERTICAL_LOWER_BOUND    0.01

int loadColorCloud(char * filename,ColorCloudPtr cloud){
    pcl::PointCloud<pcl::PointXYZRGBA> tmp;
    if (pcl::io::loadPCDFile(filename, tmp) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return -1;
    }
    cloud->clear();
    for(int i = 0; i<tmp.points.size(); i++){
        pcl::PointXYZRGBA p1 = tmp.points[i];
        ColorPoint p2;
        p2.x = p1.x;
        p2.y = p1.y;
        p2.z = p1.z;
        p2.r = p1.r;
        p2.g = p1.g;
        p2.b = p1.b;
        cloud->push_back(p2);
    }
    cloud->width = tmp.width;
    cloud->height = tmp.height;
    return 1;
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
void projectTransformTranslationsToPlane(Eigen::Vector3f x, vector<Eigen::Matrix4f>* transforms, vector<int> mask){
    //project the translation component of the transforms onto a plane defined by the parameters stored in 'x'
    for(int i = 0; i<transforms->size(); i++)
    {
        if(mask[i]!=0){
            Eigen::Matrix4f t = (*transforms)[i];
            Eigen::Vector3f p(t(0,3),t(1,3),t(2,3));
            Eigen::Vector3f origin(0,0,x(2));
            Eigen::Vector3f to_p = p-origin;
            Eigen::Vector3f normal(x(0),x(1),-1);
            
            p = p-(normal*normal.dot(to_p));
            
            (*transforms)[i] <<    t(0,0),t(0,1),t(0,2),p(0),
                                t(1,0),t(1,1),t(1,2),p(1),
                                t(2,0),t(2,1),t(2,2),p(2),
                                t(3,0),t(3,1),t(3,2),t(3,3);
        }
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
void closeCloud(ColorCloudPtr master){
    //closes the cloud by adding a plate at the bottom
    ColorCloudPtr plate (new ColorCloud);

    //get the ring of points at the bottom of the cloud
    pcl::PassThrough<ColorPoint> pass;
    pass.setInputCloud (master);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (VERTICAL_LOWER_BOUND, VERTICAL_LOWER_BOUND+.05);
    pass.filter (*plate);

    //take slices of the ring and add points to fill in the cloud
    for(float x = X_LOWER_BOUND; x<=X_UPPER_BOUND; x+=.001){
        //get just the stip contained by this slice
        ColorCloud strip;
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
            ColorPoint p;
            p.x = x;
            p.y = y;
            p.z = VERTICAL_LOWER_BOUND;
            p.r = 255;
            p.g = 255;
            p.b = 255;
            master->push_back(p);
        }
    }
}

int main(int argc, char*argv[]){
    ColorCloudPtr current (new ColorCloud); //buffer to hold the cloud for the current frame
    ColorCloudPtr master (new ColorCloud); //buffer to hold the concatenated point cloud of all frames
    ColorCloudPtr filtered (new ColorCloud); //buffer to place filtered intermediate clouds

    //keep track of all the frames
    std::vector<Eigen::Matrix4f> transforms; //the transform of the checkerboard for the frame
    std::vector<ColorCloud> clouds; //the cloud for a frame
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
            if(loadColorCloud(argv[i],current) == -1)
                return -1;
            clouds.push_back(*current);
            total_clouds ++;
            //update the user of the progress
            if(total_clouds%30==0)
                cout << (float)total_clouds/argc * 100 << " percent complete..." << endl;

            //calculate the transform of the board
            Eigen::Matrix4f transform;
            int found = getChessBoardPose(current,6,9,.0172,transform);
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
        projectTransformTranslationsToPlane(x,&transforms,success_mask); //project the translations onto the plane

        for(int i = 0; i<clouds.size(); i++)
        {
            if(success_mask[i]!=0){
                Eigen::Matrix4f transform = getSmoothedTransform(transforms,i,0.6,0,success_mask); //exponential smooth of the transforms - 0 means no smoothing
                cout << "Transform: " << transform(0,3) << " " << transform(1,3) << " " << transform(2,3) << " " << endl; //print the transform for debuging

                //transform the cloud and concatenate it with "master"
                ColorCloudPtr c (new ColorCloud(clouds[i]));
                pcl::transformPointCloud(*c,*c,transform);
                for(int i = 0; i<c->points.size(); i++){
                    pcl::PointXYZRGB p1 = c->points[i];
                    ColorPoint p2;
                    p2.x = p1.x;
                    p2.y = p1.y;
                    p2.z = p1.z;
                    p2.r = p1.r;
                    p2.g = p1.g;
                    p2.b = p1.b;
                    master->push_back(p2);
                }
            }
        }

        /*
        //print out the clouds that failed to reorient
        cout << "Fails: " << endl;
        for(int i = 0; i<fails.size(); i++)
            cout << fails[i] << endl;
        */

        //calculate the success percentage
        float percent = (((float)found_boards)/total_clouds)*100.;
        cout << "Found checkerboard in " << found_boards << " of the " << total_clouds << " clouds provided - (" << percent << "%)." << endl;
        //filter the remaining "master" cloud

        //filter by location
        cout << "Filtering the master cloud by relative position to the checkboard..." << endl;

        //strip out points not in the correct region
        pcl::PassThrough<ColorPoint> pass;
        pass.setInputCloud (master);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (VERTICAL_LOWER_BOUND, 1.0);
        pass.filter (*filtered);
        *master = *filtered;

        pass.setInputCloud (master);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (Y_LOWER_BOUND, Y_UPPER_BOUND);
        pass.filter (*filtered);
        *master = *filtered;

        pass.setInputCloud (master);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (X_LOWER_BOUND, X_UPPER_BOUND);
        pass.filter (*filtered);
        *master = *filtered;

        //filter by statical outliers
        cout << "Filtering the master cloud for statistical outliers..." << endl;
        pcl::StatisticalOutlierRemoval<ColorPoint> sor;
        sor.setInputCloud (master);
        sor.setMeanK (70);
        sor.setStddevMulThresh (3.0);
        sor.filter (*filtered);
        *master = *filtered;
        
        //downsample the cloud
        cout << "Downsampling the cloud..." << endl;
        float leaf_size = 0.002f;
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*master,*filtered,index);
        pcl::VoxelGrid<ColorPoint> vg;
        vg.setInputCloud (master);
        vg.setLeafSize (leaf_size,leaf_size,leaf_size);
        vg.filter (*filtered);
        *master = *filtered;

        pcl::io::savePCDFileASCII ("intermediate2.pcd", *master);

        //do a movingleastsquares smooth of the points
        cout << "Reconstructing..." << endl;
        pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);
        pcl::PointCloud<ColorPoint> mls_points;
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::MovingLeastSquares<ColorPoint, pcl::PointNormal> mls;
        mls.setInputCloud (master);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.01);
        mls.setOutputNormals(mls_normals);
        mls.reconstruct (mls_points);
        *master = mls_points;

        //compute the normals of the points
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<ColorPoint, pcl::Normal> ne;
        ne.setInputCloud (master);
        pcl::search::KdTree<ColorPoint>::Ptr tree2 (new pcl::search::KdTree<ColorPoint>);
        ne.setSearchMethod (tree2);
        ne.setRadiusSearch (0.01);
        ne.compute (*normals);
        
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
        closeCloud(master);

        //save the output to a file
        pcl::io::savePCDFileASCII ("test.pcd", *master);
        pcl::io::savePCDFileASCII ("normals.pcd", *normals);
    }
    return (0);
}
