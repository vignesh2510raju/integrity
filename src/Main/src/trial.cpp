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
#include "/home/vignesh/pcl-proyect/src/Main/header/read_matrices_pose.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/read_velo_to_cam.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/read_transformations.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/visualize.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/user_input.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/extracting_far_away_points.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/extracting_voxel_grid.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/cluster_extraction.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/plane_from_cluster.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/cylinder_segmentation.h"
#include "/home/vignesh/pcl-proyect/src/Main/header/store_values_in_vector_of_maps.h"

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
  
  int counter= 15; //Counter for number of PCD files that need to be executed


  float number_of_cylinders, number_of_matched_lm , number_of_expected_lm , number_of_readings;

  std::map< int , std::vector<float> > mapping_landmarks;
  std::map< int , std::vector<float> > mapping_coord_landmarks;
  std::vector<float> storeco;

  // User input
  int j, min_cluster_size, min_plane_size, min_cylinder_size; 
  bool options_flag= false;
  std::string filename;
  float max_radius, nd_weight, min_cluster_distance, min_density, xlim, ylim, zlim;  

  max_radius= 0.12f;            
  nd_weight= 0.1f;              
  min_cluster_size= 50;         
  min_plane_size= 250;           
  min_cluster_distance= 0.15;    
  min_density= 400;             
  min_cylinder_size= 25;        
  xlim= 14;                     
  ylim= 14;                     
  zlim= 0.5;

  // Parameters
  bool SWITH_VOXEL_GRID= false; // Downsample the point cloud
  bool SWITH_WRITE_CLUSTERS= false; // Write the clusters to disk 

  // To get the pose
  std::vector<Eigen::Matrix4d> T;
  T = read_transformations();  
  
 for (int fi= 0; fi <= counter; ++fi)
 {
 
   std::string filename2; 
  std::stringstream sa;
  sa << setw(6) << setfill('0') << fi;
  filename2= sa.str();

  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "File currently being used :"<< filename2 << ".pcd" << endl;
  reader.read ("../pcl_files/" + filename2 + ".pcd", *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Eliminate far-away points
  cloud = extracting_far_away_points(cloud, xlim, ylim, zlim);

  // Create the filtering object: downsample the dataset using a leaf size of 0.75cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered = extracting_voxel_grid(cloud, SWITH_VOXEL_GRID);

  // Creating the KdTree object for the search method of the extraction of clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  //tree->setInputCloud (cloud_filtered);

  // Clustering 
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  clusters = cluster_extraction (cloud_filtered, min_cluster_distance, min_cluster_size, SWITH_WRITE_CLUSTERS);

  // Extract Planes from clusters
  clusters = plane_from_cluster(clusters, min_cluster_size, min_plane_size, min_density);

 /*// Cylinder Segmentation 
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinders;
  cylinders = cylinder_segmentation(clusters, nd_weight, max_radius, min_cylinder_size, counter, fi);*/

  // Cylinder Segmentation
  float imagearray[counter][50];

  float number_of_cylinders, number_of_matched_lm , number_of_expected_lm , number_of_readings;

  std::map< int , std::vector<float> > mapping_landmarks;
  std::map< int , std::vector<float> > mapping_coord_landmarks;

  // Fit cylinders to the remainding clusters
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normals; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  // Estimate point normals
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod (tree);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg_normals.setOptimizeCoefficients (true);
  seg_normals.setModelType (pcl::SACMODEL_CYLINDER);
  seg_normals.setMethodType (pcl::SAC_RANSAC);
  seg_normals.setNormalDistanceWeight (nd_weight); // 0.1
  seg_normals.setMaxIterations (10000);
  seg_normals.setDistanceThreshold (0.2); // 0.2
  seg_normals.setRadiusLimits (0.0, max_radius); // 0.1
  seg_normals.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  seg_normals.setEpsAngle (pcl::deg2rad (15.0f));

  // Obtain the cylinder inliers and coefficients for each cluster (just one cylinder per cluster)
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > cylinders;

  int j, k = 0;

  j= 0;
  while(j < clusters.size() )
  {
    // allocate new memory to the cloud cylinder, the one before has been saved to cloud cylinder
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());

    if (clusters[j] != NULL)
    {
      // compute normals
      ne.setInputCloud (clusters[j]);
      ne.setKSearch (50); // 50
      ne.compute (*cloud_normals);

      // Segment cylinder
      seg_normals.setInputCloud (clusters[j]);
      seg_normals.setInputNormals (cloud_normals);
      seg_normals.segment (*inliers_cylinder, *coefficients_cylinder);

      // Extract indices
      extract.setInputCloud (clusters[j]);
      extract.setIndices (inliers_cylinder);
      extract.setNegative (false);
      extract.filter (*cloud_cylinder);
      if (cloud_cylinder->points.empty ()) 
      {
        //std::cerr << "Can't find the cylindrical component." << std::endl;
        cylinders[j].reset();
      }
      else
      {
        std::cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        number_of_cylinders += 1 ;
        // std::cout<< "Coefficients of the cylinder: "<< *coefficients_cylinder << endl;
        if (cloud_cylinder->points.size() > min_cylinder_size)
        {
          cylinders[j]= cloud_cylinder;

          pcl::CentroidPoint<pcl::PointXYZ> centroid;

          for (int z = 0; z < min_cylinder_size ; z++)
          {
              centroid.add (pcl::PointXYZ ( cloud_cylinder->points[ j + z ].x , cloud_cylinder->points[ j + z ].y , cloud_cylinder->points[ j + z ].z ));              
          }
          pcl::PointXYZ cl;
          centroid.get (cl);
          
          //Storing the values of the Centroid into the Image Array Matrix
          imagearray[fi][k] = ((int)(cl.x * 10 + 0.5) / 10.0 ); //rounding off to 1 decimal place
          imagearray[fi][k+1] = ((int)(cl.y * 10 + 0.5) / 10.0 ); //rounding off to 1 decimal place

          std::cout << "Centroid is : " << imagearray[fi][k] << ',' << imagearray[fi][k+1] << ' ' << std::endl;

          //Storing the values in te first map
          std::vector<float> storelm;
          storelm.push_back (1);

          //Storing the values in the Main map
          std::vector<float> storeco;
          storeco.push_back (imagearray[fi][k]);
          storeco.push_back (imagearray[fi][k+1]);
          storeco.push_back (number_of_readings);
          mapping_coord_landmarks.insert ( std::pair< int , std::vector<float> > ( fi, storeco ) );

          /*//Storing the values in the second map
          std::vector<float> storeval;
          storeval.push_back ();
          storeval.push_back ();
          storeval.push_back ();          
          mapping_landmarks.insert ( std::pair< int , std::vector<float> > ( fi, storeval ) );*/          
          

          k = k+2;

          double dist;
          pcl::PointXYZ dist2;
          dist2.x = 0.0;
          dist2.y = 0.0;
          dist2.z = 0.0;
          dist = pcl::euclideanDistance ( cl , dist2 );
          dist = ((int)(dist * 10 + 0.5) / 10.0 ); 
          std::cout << "Distance from the car - " << dist << std::endl;
        }
        else
        {
          cylinders[j].reset();
        }

      }
    }

    j++;
  }

  //Number of cylidners
  std::cout << std::endl << std::endl << "Number of cylinders : " << number_of_cylinders << std::endl << std::endl ;

  //Storing the values in the second map
  std::vector<float> storeval;
  storeval.push_back (number_of_cylinders);          
  mapping_landmarks.insert ( std::pair< int , std::vector<float> > ( fi, storeval ) );

  // Visualization
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  //viewer.setFullScreen(true); 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
  viewer.addPointCloud <pcl::PointXYZ>(cloud_filtered, white_color, "cloud");
  viewer = visualize(viewer, cylinders, "cylinders", true, true);

  // Run the viewer
  //while (!viewer.wasStopped ())
  // {
     viewer.spinOnce (0.01);
  // }
 }  
  return (0);
}