#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include <pwd.h>
#include "../header/read_matrices_pose.h"
#include "../header/read_velo_to_cam.h"
#include "../header/read_transformations.h"
#include "../header/visualize.h"
#include "../header/user_input.h"
#include "../header/extracting_far_away_points.h"
#include "../header/extracting_voxel_grid.h"
#include "../header/cluster_extraction.h"
#include "../header/plane_from_cluster.h"
#include "../header/cylinder_segmentation.h"
#include "../header/store_values_in_vector_of_maps.h"

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

int main (int argc, char** argv)
{
  
  // int numFrames= 15; //Counter for number of PCD files that need to be executed


  int number_of_matched_lm , number_of_expected_lm , number_of_readings, j=0;

  // Vector to store the values in a map
  // std::vector<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > mapp;
  
  Retval value, value2;

  value.number_of_cylinders = 0;

  std::string cloud_name;

  // User input
  int min_cluster_size, min_plane_size; 
  bool options_flag= false;
  std::string filename;
  float min_cluster_distance, min_density, xlim, ylim, zlim;  

  value.numFrames = 8;

  value.max_radius= 0.10f;            
  value.nd_weight= 0.1f;              
  min_cluster_size= 30;         
  min_plane_size= 300;           
  min_cluster_distance= 0.15;    
  min_density= 400;             
  value.min_cylinder_size= 25;
  xlim= 14;                     
  ylim= 14;                     
  zlim= 0.5;

  // Parameters
  bool SWITH_VOXEL_GRID= true; // Downsample the point cloud
  bool SWITH_WRITE_CLUSTERS= false; // Write the clusters to disk 

  // To get the pose
  std::vector<Eigen::Matrix4d> T;
  T = read_transformations();  
  
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_original (new pcl::PointCloud<pcl::PointXYZ>);

 // pcl::visualization::PCLVisualizer viewer("PCL Viewer"); 

 for (value.i= 0; value.i <= value.numFrames; ++value.i)
 {
 
  std::string filename2; 
  std::stringstream sa;
  sa << setw(6) << setfill('0') << value.i;
  filename2= sa.str();

  // Read in the cloud data
  pcl::PCDReader reader;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "File currently being used :"<< filename2 << ".pcd" << endl;
  reader.read ("../Data/pcd-files/Test/" + filename2 + ".pcd", *cloud_original);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 0.75cm
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  extracting_voxel_grid(cloud_original, cloud, SWITH_VOXEL_GRID);

  // Eliminate far-away points
  extracting_far_away_points(cloud, xlim, ylim, zlim);

  // Clustering 
  // std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  value.clusters = cluster_extraction (cloud, min_cluster_distance, min_cluster_size, SWITH_WRITE_CLUSTERS);

  // Extract Planes from clusters
  // clusters = plane_from_cluster(clusters, min_cluster_size, min_plane_size, min_density);

  // Cylinder Segmentation 
  // std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinders;

  // std::tie( cylinders, number_of_cylinders ) = cylinder_segmentation(clusters, nd_weight, max_radius, min_cylinder_size, numFrames, i);
  
  value2 = cylinder_segmentation(value); 

  // std::cout << "try print cylinders ===== " << number_of_cylinders << std::endl;

  // Storing values 
  // store_values_in_vector_of_maps (cylinders, i, );

  // Visualization
  // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  // viewer.setFullScreen(true); 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
  viewer.addPointCloud <pcl::PointXYZ>(cloud, white_color, "cloud_" + patch::to_string((value.i*10)+value.i));
  // viewer = visualize(viewer, value2.cylinders, "cylinders", true, value.i);
  // viewer.removePointCloud (cloud, white_color, "cloud_" + patch::to_string((value.i*10)+value.i));
  // viewer.removeAllPointClouds ();
  // viewer.removePointCloud ( cloud , 0);
  // viewer.removeAllPointClouds ( value.i );

  j=0;
  while(j <  value2.cylinders.size())
  {
    if ( value2.cylinders[j] != NULL)
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color ( value2.cylinders[j], 255, 0, 0);

      cloud_name= "cloud" + patch::to_string((value.i*10)+j);

      // if (OPT_RED)
      // {
      //   viewer.addPointCloud<pcl::PointXYZ> (newClouds[j], red_color, cloud_name);
      // }
      // else
      // {
      if ( j == 0)
      {
       viewer.addPointCloud<pcl::PointXYZ> (value2.cylinders[j], red_color, cloud_name);
      }
      else {
        viewer.updatePointCloud<pcl::PointXYZ> (value2.cylinders[j], red_color, cloud_name);
      }
    }
    j++;
  }

      /* Initialize the veiwer */ 
    // pcl::visualization::PCLVisualizer viewer ("Point Cloud"); 

    // while(!viewer.wasStopped()) 
    // { 
    //         /* play with your data here. */ 
    //         viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
    //         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
    //         viewer.addPointCloud <pcl::PointXYZ>(cloud, white_color, "cloud_" + patch::to_string((value.i*10)+value.i));
    //         viewer = visualize(viewer, value2.cylinders, "cylinders", true, value.i);
  
    //         viewer.spinOnce(75); 
    // } 

  // Run the viewer
  // while (!viewer.wasStopped ())
  // {
     viewer.spinOnce (75);
  // }
 }  


  return (0);
}
