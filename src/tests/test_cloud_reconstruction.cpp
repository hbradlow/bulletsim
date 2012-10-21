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
#include "clouds/model_building.cpp"

using namespace std;

int main(int argc, char*argv[]){
    ColorCloudPtr tmp (new ColorCloud); //a buffer to store the cloud that is loaded from disk
    pcl::PointCloud<pcl::PointNormal>::Ptr current (new pcl::PointCloud<pcl::PointNormal>); //buffer to hold the cloud for the current frame

    pcl::PointCloud<pcl::PointNormal>::Ptr master (new pcl::PointCloud<pcl::PointNormal>); //buffer to hold the concatenated point cloud of all frames
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered (new pcl::PointCloud<pcl::PointNormal>);//buffer to place filtered intermediate clouds

    //keep track of all the frames
    std::vector<Eigen::Matrix4f> transforms; //the transform of the checkerboard for the frame
    std::vector< pcl::PointCloud<pcl::PointNormal> > clouds; //the cloud for a frame
    std::vector< pcl::PointCloud<pcl::PointNormal> > clouds_raw; //the cloud for a frame
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
            if(total_clouds%2==0)
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

        pcl::PointCloud<pcl::PointNormal>::Ptr master_raw (new pcl::PointCloud<pcl::PointNormal>);
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

                //compute normals of the clouds
                pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                pcl::NormalEstimation<pcl::PointNormal, pcl::Normal> ne;
                ne.setInputCloud (c);
                pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
                ne.setSearchMethod (tree);
                ne.setRadiusSearch (0.01);
                ne.compute (*normals);
                for(int j = 0; j<c->points.size(); j++){
                    pcl::PointNormal* p1 = &(c->points[j]);
                    pcl::Normal n1 = normals->points[j];
                    p1->normal_x = n1.normal_x;
                    p1->normal_y = n1.normal_y;
                    p1->normal_z = n1.normal_z;
                }
            
                pcl::transformPointCloudWithNormals(*c,*c,transform);
                filterByLocation(c,c);

                clouds_raw.push_back(*c);
                for(int j = 0; j<c->points.size(); j++){
                    pcl::PointNormal p1 = c->points[j];
                    master_raw->push_back(p1);
                }
            }
        }

        //calculate the success percentage
        float percent = (((float)found_boards)/total_clouds)*100.;
        cout << "Found checkerboard in " << found_boards << " of the " << total_clouds << " clouds provided - (" << percent << "%)." << endl;
        //filter the remaining "master" cloud

        /*
        //try to align the clouds better using ICP
        for(int i = 0; i< clouds_raw.size(); i++){
            pcl::PointCloud<pcl::PointNormal>::Ptr c (new pcl::PointCloud<pcl::PointNormal>(clouds_raw[i]));
            Eigen::Matrix4f transform;
            cout << "HERE" << endl;
            if(getTransformForICPAlignment(master_raw,c,&transform)){
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
        *master = *master_raw;

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

        //close the bottom of the cloud to make a semi inclosed surface
        cout << "Closing the bottom of the shape..." << endl;
        closeCloud(master);

        //save the output to a file
        pcl::io::savePCDFileASCII ("output.pcd", *master);
    }
    return (0);
}
