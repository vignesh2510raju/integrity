#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }

  std::cout << "points: " << cloud->points.size () << std::endl;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  normalEstimation.setRadiusSearch (0.5);

  // Compute the features
  normalEstimation.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
  std::cout << "cloud_normals->points.size (): " << cloud_normals->points.size () << std::endl;
  //return 0;

  // Open 3D viewer and add point cloud and normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addPointCloud <pcl::PointXYZ>(cloud, "cloud");
  viewer.addPointCloudNormals <pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 10, 0.9, "normals");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();
  
  //writing normal (x,y,z) into a PCD file
  pcl::copyPointCloud(*cloud, *cloud); 
  pcl::copyPointCloud(*cloud_normals, *cloud_normals); 

  pcl::io::savePCDFileASCII("table_scene_mug_stereo_textured_normal.pcd", *cloud_normals);

  // Display the result
  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return (0);
}
