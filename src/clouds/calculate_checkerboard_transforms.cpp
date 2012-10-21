#include "clouds/model_building.cpp"
using namespace std;

int main(int argc, char*argv[]){
    ColorCloudPtr tmp (new ColorCloud); //a buffer to store the cloud that is loaded from disk
    pcl::PointCloud<pcl::PointNormal>::Ptr current (new pcl::PointCloud<pcl::PointNormal>); //buffer to hold the cloud for the current frame

    //keep track of all the frames
    vector<Eigen::Matrix4f> transforms; //the transform of the checkerboard for the frame
    vector<int> success_mask; //indicates if a checkerboard was located in the frame

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
            
            total_clouds ++;
            //update the user of the progress
            if(total_clouds%2==0)
                cout << "PERCENT COMPLETE:" << (float)total_clouds/argc * 100 << endl;

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
        writeTransforms("transforms.txt",&transforms,&success_mask);
        /*
        //project the transforms onto a plane
        Eigen::Vector3f x = fitPlaneToTransformTranslations(transforms,success_mask); //get the parameters of the best fit plane to the checkerboard translations
        for(int i  = 0; i< transforms.size(); i++){
            //transforms[i] = projectTransformToPlane(transforms[i],x);
        }
        projectTransformTranslationsToPlane(x,&transforms,success_mask); //project the translations onto the plane
        */
    }
    return 0;
}
