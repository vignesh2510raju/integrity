#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("../pcl_files/000000.pcd", *cloud);

  std::cout << "Cloud before filtering: " << cloud->width * cloud->height <<  std::endl;

  // Downsample
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud_filtered);
  *cloud= *cloud_filtered;
  std::cout << "Cloud after filtering: " << cloud->width * cloud->height <<  std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (atof(argv[1])); // 30
  pmf.setSlope (atof(argv[2])); // 1.0f
  pmf.setInitialDistance (atof(argv[3])); // 0.1
  pmf.setMaxDistance (atof(argv[4])); // 2
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  // Extract non-ground returns
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  // writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);

  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color (cloud, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (cloud, 255, 0, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud <pcl::PointXYZ>(cloud, white_color, "cloud");
  viewer.addPointCloud <pcl::PointXYZ>(cloud_filtered, red_color, "cloud_filtered");
 
  //Write the non-ground points to another PCD file
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("../pcl_files/000000_nonground.pcd", *cloud_filtered, false);

  // Display the result
  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}
