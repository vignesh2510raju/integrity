#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

// To string function
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

  // Parameters
  std::string filename = argv[1];
  int min_points= atoi(argv[2]); // 1000;
  float distance_threshold= atof(argv[3]);  // 0.30;
  int number_iterations= atoi(argv[4]); // 700;

  // Load the data
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }
  std::cout << "points before filtering: " << cloud->points.size () <<std::endl;

  // store the coefficients in a map of pcl::ModelCoefficients
  std::map<int, pcl::ModelCoefficients> plane_store;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  
  // Optional
  seg.setOptimizeCoefficients (true);
  
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold); // how close to be inlier
  seg.setEpsAngle (pcl::deg2rad (15.0f));
  seg.setMaxIterations(number_iterations);
  
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color (cloud, 255, 255, 255);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud <pcl::PointXYZ>(cloud, cloud_color, "cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_plane_green (cloud_plane, 0, 255, 0);
  
  // extract the ground plane and remove those points
  seg.setDistanceThreshold (0.20); // how close to be inlier
  seg.setAxis (Eigen::Vector3f (0.0, 1.0, 0.0));
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud);
  cloud.swap (cloud);

  //set the vertical axis again and augment the threshold
  seg.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  seg.setDistanceThreshold (distance_threshold); // how close to be inlier

  int i = 0, nr_points = (int) cloud->points.size ();
  
  // While 30% of the original cloud is still there
  while (cloud->points.size () > 0.3 * nr_points)
  {
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    // eliminate the points extracted from the cloud
    extract.setNegative (true);
    extract.filter (*cloud);
    cloud.swap (cloud);

    // create viwer name
    std::string view_name= "plane" + patch::to_string(i);

    std::cout<< "cloud number: "<< view_name<< "  Number of points: "<< inliers->indices.size()<< std::endl;

    if (inliers->indices.size() > min_points)
    {
      // store the plane in the map
      plane_store[i]= *coefficients;
      viewer.addPointCloud <pcl::PointXYZ>(cloud_plane, cloud_plane_green, view_name);
      i++;    
    }
  }

  std::cout<< "END"<< std::endl;

  for (int j = 0; j < i; ++j)
  {
    std::cerr << "Model coefficients: " << plane_store[j].values[0] << " " 
                                        << plane_store[j].values[1] << " "
                                        << plane_store[j].values[2] << " " 
                                        << plane_store[j].values[3] << std::endl;  
  }

  // Display the result
  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return (0);
}