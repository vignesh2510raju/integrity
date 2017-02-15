#include "../header/extracting_voxel_grid.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

void extracting_voxel_grid (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original, 
                                                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                                                           bool SWITH_VOXEL_GRID)
{
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  if (SWITH_VOXEL_GRID)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_original);
    vg.setLeafSize (0.095f, 0.095f, 0.095f);
    vg.filter (*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 
  }
  else
  {
    *cloud_filtered= *cloud_original;
    std::cout << "PointCloud not filted - same as original cloud" << std::endl; 
  }
}