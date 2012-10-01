#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include "clouds/utils_pcl.h"

#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/registration/transforms.h>
#include "clouds/pcl_typedefs.h"
#include "clouds/get_chessboard_pose.h"
#include "clouds/cloud_ops.h"

using namespace std;

int main(int argc, char*argv[]){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>); //to load the file
    ColorCloudPtr current (new ColorCloud); //to process the file

    ColorCloudPtr master (new ColorCloud);
    ColorCloudPtr filtered (new ColorCloud);
    std::vector<int> index;

    if(argc>1){
        // Reorient each cloud to line the checkerboards up and concatenate them all into a "master" cloud
        int found_boards = 0;
        int total_clouds = 0;
        cout << "Reorienting the clouds to match their checkerboards..." << endl;
        vector<char *> fails;

        for(int i=1; i<argc; i++)
        {
            if (pcl::io::loadPCDFile(argv[i], *current_tmp) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return (-1);
            }
            current->clear();
            for(int i = 0; i<current_tmp->points.size(); i++){
                pcl::PointXYZRGBA p1 = current_tmp->points[i];
                ColorPoint p2;
                p2.x = p1.x;
                p2.y = p1.y;
                p2.z = p1.z;
                p2.r = p1.r;
                p2.g = p1.g;
                p2.b = p1.b;
                current->push_back(p2);
            }
            current->width = current_tmp->width;
            current->height = current_tmp->height;

            Eigen::Matrix4f transform;
            //int found = getChessBoardPose(current,4,9,.0208,transform);
            int found = getChessBoardPose(current,6,9,.0172,transform);
            total_clouds ++;
            if(total_clouds%30==0)
                cout << (float)total_clouds/argc * 100 << " percent complete..." << endl;
            if(found){
                found_boards ++;
                pcl::transformPointCloud(*current,*current,transform);


                if(master->size()>0){
                    for(int i = 0; i<current->points.size(); i++){
                        pcl::PointXYZRGB p1 = current->points[i];
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
            else
                fails.push_back(argv[i]);
            if(master->size()==0)
                *master = *current;
        }
        cout << "Fails: " << endl;
        for(int i = 0; i<fails.size(); i++)
            cout << fails[i] << endl;
        float percent = (((float)found_boards)/total_clouds)*100.;
        cout << "Found checkerboard in " << found_boards << " of the " << total_clouds << " clouds provided - (" << percent << "%)." << endl;
        //filter the remaining "master" cloud
        /*
        float x_lower_bound = -.1;
        float x_upper_bound = -.5;
        float y_lower_bound = -.1;
        float y_upper_bound = .1;
        */
        float x_lower_bound = -.33;
        float x_upper_bound = -.18;
        float y_lower_bound = -.1;
        float y_upper_bound = .1;
        float vertical_lower_bound = .01;

        //filter by location
        cout << "Filtering the master cloud by relative position to the checkboard..." << endl;
        pcl::PassThrough<ColorPoint> pass;
        pass.setInputCloud (master);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (vertical_lower_bound, 1.0);
        pass.filter (*filtered);
        *master = *filtered;

        pass.setInputCloud (master);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (y_lower_bound, y_upper_bound);
        pass.filter (*filtered);
        *master = *filtered;

        pass.setInputCloud (master);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x_lower_bound, x_upper_bound);
        pass.filter (*filtered);
        *master = *filtered;

        pcl::io::savePCDFileASCII ("intermediate.pcd", *master);

        //filter by statical outliers
        cout << "Filtering the master cloud for statistical outliers..." << endl;
        pcl::StatisticalOutlierRemoval<ColorPoint> sor;
        sor.setInputCloud (master);
        sor.setMeanK (70);
        sor.setStddevMulThresh (3.0);
        sor.filter (*filtered);
        *master = *filtered;
        
        pcl::io::savePCDFileASCII ("intermediate2.pcd", *master);

        //downsample the cloud to speed up later filters
        cout << "Downsampling the cloud..." << endl;
        pcl::removeNaNFromPointCloud(*master,*filtered,index);
        float downsample = 0.002f;
        pcl::VoxelGrid<ColorPoint> vg;
        vg.setInputCloud (master);
        vg.setLeafSize (downsample,downsample,downsample);
        vg.filter (*filtered);
        *master = *filtered;

        cout << "Reconstructing..." << endl;
        // Create a KD-Tree
        pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<ColorPoint> mls_points;
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals;
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_normals;
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<ColorPoint, pcl::PointNormal> mls;
//        mls.setComputeNormals (true);
        // Set parameters
        mls.setInputCloud (master);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.01);
        mls.setOutputNormals(mls_normals);
        // Reconstruct
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
        
        /*
        cout << "Closing the bottom of the shape..." << endl;
        ColorCloudPtr plate (new ColorCloud);
        pass.setInputCloud (master);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (vertical_lower_bound, vertical_lower_bound+.05);
        pass.filter (*plate);

        for(float x = x_lower_bound; x<=x_upper_bound; x+=.001){
            ColorCloud strip;
            pass.setInputCloud (plate);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (x, x+.01);
            pass.filter (strip);
            if(strip.size()==0){
                continue;
            }

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
            cout << min << " " << max << endl;
            for(float y = min; y<=max; y+=.001){
                ColorPoint p;
                p.x = x;
                p.y = y;
                p.z = vertical_lower_bound;
                p.r = 255;
                p.g = 255;
                p.b = 255;
                master->push_back(p);
            }
        }
        */



        //save the output to a file
        pcl::io::savePCDFileASCII ("test.pcd", *master);
    }
    return (0);
}