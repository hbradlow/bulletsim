#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include "clouds/utils_pcl.h"

#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/registration/transforms.h>
#include "clouds/pcl_typedefs.h"
#include "clouds/get_chessboard_pose.h"
#include "clouds/cloud_ops.h"

using namespace std;

int main(int argc, char*argv[]){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ColorCloudPtr current (new ColorCloud);
    ColorCloudPtr master (new ColorCloud);
    ColorCloudPtr filtered (new ColorCloud);
    std::vector<int> index;
    if(argc>1){
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
            int found = getChessBoardPose(current,6,9,.0172,transform);
            if(found){
                cout << found << endl;
                cout << transform << endl;
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
            if(master->size()==0)
                *master = *current;
        }
        float horizontal_bound = .5;
        float vertical_lower_bound = .05;

        //filter by location
        pcl::PassThrough<ColorPoint> pass;
        pass.setInputCloud (master);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (vertical_lower_bound, 1.0);
        pass.filter (*filtered);
        *master = *filtered;

        pass.setInputCloud (master);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-horizontal_bound, horizontal_bound);
        pass.filter (*filtered);
        *master = *filtered;

        pass.setInputCloud (master);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-horizontal_bound, horizontal_bound);
        pass.filter (*filtered);
        *master = *filtered;

        /*
        //filter by statical outliers
        pcl::StatisticalOutlierRemoval<ColorPoint> sor;
        sor.setInputCloud (master);
        sor.setMeanK (1000);
        sor.setStddevMulThresh (1.0);
        sor.filter (*filtered);
        *master = *filtered;
        */

        // Create a KD-Tree
        pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<ColorPoint> mls_points;
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<ColorPoint, pcl::PointNormal> mls;
//        mls.setComputeNormals (true);
        // Set parameters
        mls.setInputCloud (master);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.03);
        // Reconstruct
        mls.reconstruct (mls_points);
        *master = mls_points;
        
        /*
        //downsample the cloud
        pcl::removeNaNFromPointCloud(*master,*filtered,index);
        float downsample = 0.00001f;
        pcl::VoxelGrid<ColorPoint> sor;
        sor.setInputCloud (filtered);
        sor.setLeafSize (downsample,downsample,downsample);
        sor.filter (*master);
        */

        pcl::io::savePCDFileBinary ("test.pcd", *master);
    }
    return (0);
}
