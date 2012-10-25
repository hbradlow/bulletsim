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
    std::vector<int> success_mask; //indicates if a checkerboard was located in the frame
    readTransforms("transforms.txt",&transforms,&success_mask);

    if(argc>1){
        // Reorient each cloud to line the checkerboards up and concatenate them all into a "master" cloud
        cout << "Reorienting the clouds to match their checkerboards..." << endl;
        //load all of the clouds into the "clouds" vector
        for(int i=1; i<argc; i++)
        {
            //load the cloud from the file
            if(loadCloud(argv[i],current,tmp) == -1)
                return -1;
            clouds.push_back(*current);
        }

        for(int i = 0; i<clouds.size(); i++)
        {
            if(success_mask[i]!=0){

                cout << "PERCENT COMPLETE:" << (float)i/clouds.size() * 100 << endl;

                Eigen::Matrix4f transform = transforms[i]; 

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

                for(int j = 0; j<c->points.size(); j++){
                    pcl::PointNormal p1 = c->points[j];
                    master->push_back(p1);
                }
            }
        }
        
        //pcl::io::savePCDFileASCII ("intermediate.pcd", *master);


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

