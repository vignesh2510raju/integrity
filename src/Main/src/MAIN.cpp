
/* Main code. The process followed here is explained in the README. */

#include "../header/MAIN.h"
#include "../header/classes.h"
#include "../header/read_matrices_pose.h"
#include "../header/read_velo_to_cam.h"
#include "../header/read_transformations.h"
#include "../header/visualize.h"
#include "../header/user_input.h"
#include "../header/extracting_far_away_points.h"
#include "../header/extracting_voxel_grid.h"
#include "../header/cluster_extraction.h"
#include "../header/plane_from_cluster.h"
#include "../header/plane_from_cluster_2.h"
#include "../header/cylinder_segmentation.h"
#include "../header/store_values_in_vector_of_maps.h"
#include "../header/Inputs.h"
#include "../header/UpdateMAP_saveFrame.h"


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

//----------------------------  MAIN  ----------------------------//
int main (int argc, char** argv)
{

  Parameters parameters;
  std::vector <Frame> frames;
  std::vector <Landmark> landmarks;

  // User input
  bool options_flag= false;
  std::string filename;

  parameters= GetUserInputs(argc, argv, parameters);

  // Writing the xy pose of the cylinders extracted in the local car frame to a text file
  ofstream outputFile("output_for_matlab.txt");

  // To get the pose
  std::vector<Eigen::Matrix4d> T= read_transformations();  

  // Need two clouds, original cloud to plot and cloud to work on
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
  								 cloud_original (new pcl::PointCloud<pcl::PointXYZ>);

  // clusters
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  std::vector <Cylinder> cylinders;

  // Initialize viwer
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  // viewer.setFullScreen(true); 
  viewer.setCameraPosition(-21.1433, -23.4669, 12.7822,  // do not change unless you know how
                           0.137915, -0.429331, -1.9301,
                           0.316165, 0.28568, 0.904669);
  viewer.setCameraClipDistances(0.0792402, 79.2402); 
  std::string cloud_cylinder_id;

  for (int i= 0; i <= parameters.numFrames; ++i)
  {

  	//----------------------------  READ DATA  ----------------------------//
  	// Form the name to read the data
  	std::string filename2; 
  	std::stringstream sa;
  	sa << setw(6) << setfill('0') << i;
  	filename2= sa.str();

  	// Read in the cloud data
  	pcl::PCDReader reader;
  	std::cout << "File currently being used :"<< filename2 << ".pcd" << endl;
  	reader.read ("../Data/pcd-files/KITTI/" + filename2 + ".pcd", *cloud_original);
  	std::cout << "PointCloud before filtering has: " 
              << cloud_original->points.size () << " data points." << std::endl; //*
  	//----------------------------  READ DATA  ----------------------------//


  	// Create the filtering object: downsample the dataset using a leaf size of 0.75cm
  	extracting_voxel_grid(cloud_original, cloud, parameters.SWITH_VOXEL_GRID);

  	// Eliminate far-away points
  	extracting_far_away_points(cloud, parameters);

  	// Clustering 
  	clusters.clear(); // clear the clusters to store new ones
  	cluster_extraction (cloud, clusters, parameters);

  	// Extract Planes from clusters so that it is easier to find the cylinders
  	plane_from_cluster (clusters, parameters);

    // Cylinder Segmentation 
   	// std::vector <Cylinder> cylinders;
   	cylinders.clear();
    cylinder_segmentation( cylinders, clusters, T[i], parameters ); 

    // Update MAP and save frame
    UpdateMAP_saveFrame( cylinders, frames, landmarks, T[i], parameters );


    //----------------------------  VISUALIZATION  ----------------------------//
    // Remove previous point clouds
    viewer.removeAllPointClouds();

    // Update cloud original - White cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
                                    white_color (cloud_original, 255, 255, 255);                                
    viewer.addPointCloud <pcl::PointXYZ>(cloud_original, white_color, 
                                                       "cloud_original");    

    // Update the cylinders
    for (std::vector< Cylinder >::iterator 
                                      it= cylinders.begin();
                                      it != cylinders.end(); ++it)
    {
      cloud_cylinder_id= "cloud_cylinder_" + 
                          patch::to_string( std::distance(cylinders.begin(), it) );
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
                          red_color ( it->cloud, 255, 0, 0);
      viewer.addPointCloud<pcl::PointXYZ> (it->cloud, red_color, cloud_cylinder_id);
      viewer.setPointCloudRenderingProperties 
                  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_cylinder_id);

      cout<< "ploting "<< cloud_cylinder_id<< " with #points "<< it->cloud->points.size()<<  endl;
    }

    viewer.spinOnce ();
    // ----------------------------  VISUALIZATION  ----------------------------//




    std::cout<< "\n \n \n \n \n \n" << std::endl; 
  }




  // Show final map
  printf("#lm\t X\t\t Y\t\t rep \n");
  for (int i = 0; i < landmarks.size(); ++i)
  {
    printf("%d)\t %.2f\t\t %.2f\t\t %d \n", i,
            landmarks[i].pose[0], landmarks[i].pose[1], landmarks[i].rep );
  }

  cout<< "\n \n \n" << std::endl; 

  // Show the repeatability per frame
  printf("#frame\t #features\t #detect\t #expect\t rep \n");
  for (int i = 0; i < frames.size(); ++i)
  {
    printf("%i)\t %d\t\t %d\t\t %d\t\t %.2f \n", i, frames[i].numFeatures,
                            frames[i].numDetected, 
                            frames[i].numExpected, frames[i].repRate);
  }
  
  cout<< "\n \n \n" << std::endl; 

  // Show the velo measurements in each frame
  for (int i = 0; i < frames.size(); ++i)
  {
    printf("%i) z_velo\t \n", i);
    for (int j = 0; j < frames[i].numFeatures ; j++)
    {
      printf("%-.2f\t  %-.2f\t %-.2f\t %-.2f\t %-.2f\t \n", 
             frames[i].z_velo[j][0], frames[i].z_velo[j][1], frames[i].z_velo[j][2],
             frames[i].z_velo[j][3], frames[i].z_velo[j][4]);
    }
  }

  cout<< "\n \n \n" << std::endl; 

  // Show the nav measurements in each frame
  cout<< "\n \n \n" << std::endl; 

  for (int i = 0; i < frames.size(); ++i)
  {
    printf("%i) z_nav\t \n", i);
    for (int j = 0; j < frames[i].numFeatures ; j++)
    {
      printf("%-.2f\t  %-.2f\t %-.5f\t %-.5f\t %-.5f\t \n", 
             frames[i].z_nav[j][0], frames[i].z_nav[j][1], frames[i].z_nav[j][2],
             frames[i].z_nav[j][3], frames[i].z_nav[j][4]);
    }
    outputFile << endl;
  }

  return (0);
}
